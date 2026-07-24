from __future__ import annotations

import time
from collections import deque
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .control_state import (
    hardware_control_ready,
    select_command_mode,
    update_home_latch,
    should_reset_vr_reference,
    tracking_data_is_fresh,
)
from .oculus_reader import OculusReader
from .piper_ik import PiperIK
from .piper_sdk_adapter import PiperSdkAdapter
from .safe_home_config import SAFE_HOME_Q
from .pose_math import (
    limit_target_translation_with_margin,
    limit_pose_error,
    average_pose_window,
    apply_wrist_pivot,
    limit_target_translation_with_wall,
    quest_to_robot_transform,
    relative_pose,
    scale_target_rotation,
    scale_target_translation,
    adaptive_pose_step,
    apply_pose_deadband,
    xyzrpy_matrix,
    limit_input_pose_jump,
    pose_jump_exceeds,
    pose_window_is_stable,
)


class QuestSinglePiperNode(Node):
    def __init__(self):
        super().__init__("quest_single_piper_node")
        self.declare_parameter("urdf_path", "")
        self.declare_parameter("can_name", "can0")
        self.declare_parameter("allow_hardware", False)
        self.declare_parameter("quest_ip", "")
        self.declare_parameter("require_b_button", True)
        self.declare_parameter("b_debounce_sec", 0.10)
        self.declare_parameter("b_release_debounce_sec", 0.80)
        self.declare_parameter("speed_rate", 10)
        self.declare_parameter("max_joint_step_rad", 0.01)
        self.declare_parameter("ik_max_joint_jump_deg", 120.0)
        self.declare_parameter("translation_scale", 1.5)
        self.declare_parameter("rotation_scale", 0.5)
        self.declare_parameter("pose_smoothing_alpha", 0.14)
        self.declare_parameter("rotation_smoothing_alpha", 0.14)
        self.declare_parameter("fast_translation_response_alpha", 0.30)
        self.declare_parameter("fast_rotation_response_alpha", 0.26)
        self.declare_parameter("pose_filter_window", 3)
        self.declare_parameter("wrist_pivot_offset_m", [0.0, 0.0, 0.0])
        self.declare_parameter("max_tracking_age_sec", 0.12)
        self.declare_parameter("b_anchor_settle_sec", 0.16)
        self.declare_parameter("b_anchor_max_translation_m", 0.01)
        self.declare_parameter("b_anchor_max_rotation_rad", 0.08)
        self.declare_parameter("pose_deadband_m", 0.002)
        self.declare_parameter("pose_deadband_rad", 0.02)
        self.declare_parameter("max_input_translation_step_m", 0.03)
        self.declare_parameter("max_input_rotation_step_rad", 0.16)
        self.declare_parameter("max_input_discontinuity_m", 0.15)
        self.declare_parameter("max_input_discontinuity_rad", 0.60)
        self.declare_parameter("adaptive_translation_threshold_m", 0.015)
        self.declare_parameter("adaptive_rotation_threshold_rad", 0.08)
        self.declare_parameter("max_cartesian_step_m", 0.02)
        self.declare_parameter("max_rotation_step_rad", 0.12)
        self.declare_parameter("workspace_min", [0.02, -0.65, 0.02])
        self.declare_parameter("workspace_max", [0.75, 0.65, 0.75])
        self.declare_parameter("workspace_margin_m", 0.03)
        self.declare_parameter("reach_limit_m", 0.15)
        self.declare_parameter("reach_limit_rad", 0.60)
        self.declare_parameter("hold_when_b_released", True)
        self.declare_parameter("home_requires_b", True)
        self.declare_parameter("disable_on_exit", False)
        urdf = self.get_parameter("urdf_path").value
        if not urdf:
            urdf = get_package_share_directory("quest_teleop_ros2") + "/urdf/piper_description.urdf"
        self.ik = PiperIK(
            urdf,
            float(self.get_parameter("ik_max_joint_jump_deg").value),
        )
        # No-hardware mode keeps the historical nominal origin; hardware mode
        # replaces this with the measured startup end-effector pose below.
        self.startup_pose = xyzrpy_matrix(0.19, 0.0, 0.2, 0, 0, 0)
        self.measured_state_synced = bool(self.get_parameter("allow_hardware").value)
        self.adapter = PiperSdkAdapter(
            self.get_parameter("can_name").value,
            bool(self.get_parameter("allow_hardware").value),
            int(self.get_parameter("speed_rate").value),
            float(self.get_parameter("max_joint_step_rad").value),
        )
        if self.adapter.allow_hardware and not self.adapter.connect():
            self.adapter.close(disable=True)
            raise RuntimeError("Piper SDK could not connect/enable the arm")
        if self.adapter.allow_hardware:
            current_q = self.adapter.read_joint_positions_rad()
            if current_q is None:
                self.adapter.close(disable=True)
                raise RuntimeError("Could not read Piper joint state before motion")
            self.ik.last_q = np.asarray(current_q, dtype=float)
            self.startup_pose = self.ik.forward_ee_pose(current_q)
            self.adapter.set_command_reference(current_q)
            self.home_q = np.asarray(SAFE_HOME_Q, dtype=float)
            self.last_valid_q = np.asarray(current_q, dtype=float)
            self.get_logger().warning(
                "Hardware enabled: starting from measured joint position; "
                "B button gate and step limiter are active"
            )
        ip = self.get_parameter("quest_ip").value or None
        self.reader = OculusReader(ip_address=ip)
        self.pub = self.create_publisher(JointState, "joint_command", 10)
        self.measured_sub = self.create_subscription(
            JointState, "piper_measured_joint_state", self._on_measured_state, 10
        )
        self.base = None
        self.home_q = getattr(self, "home_q", np.asarray(SAFE_HOME_Q, dtype=float))
        self.control_origin_pose = self.startup_pose.copy()
        self.home_latched = False
        self.last_valid_q = getattr(self, "last_valid_q", None)
        self.last_gripper = 0.0
        self.filtered_target = None
        self.pose_history = deque(
            maxlen=max(1, int(self.get_parameter("pose_filter_window").value))
        )
        self.b_pressed_since = None
        self.b_release_since = None
        self.previous_b_pressed = False
        self.previous_b_gate_active = False
        self.last_input_xyz = None
        self.last_input_pose = None
        self.input_jump_latched = False
        self.anchor_pending = False
        self.anchor_started = None
        self.anchor_history = deque(maxlen=16)
        self.timer = self.create_timer(0.02, self.tick)
        self.get_logger().warning("Piper hardware is disabled unless allow_hardware:=true")

    def _on_measured_state(self, message):
        if self.measured_state_synced or len(message.position) < 6:
            return
        q = np.asarray(message.position[:6], dtype=float)
        if not np.all(np.isfinite(q)):
            return
        self.ik.last_q = q.copy()
        self.last_valid_q = q.copy()
        self.startup_pose = self.ik.forward_ee_pose(q)
        self.control_origin_pose = self.startup_pose.copy()
        self.measured_state_synced = True
        self.get_logger().info(
            "Synchronized measured Piper state before VR control",
            throttle_duration_sec=1.0,
        )

    def tick(self):
        transforms, buttons = self.reader.get_transformations_and_buttons()
        tracking_age = self.reader.get_data_age_sec()
        right_pose_present = int("r" in transforms)
        tracking_fresh = tracking_data_is_fresh(
            tracking_age, float(self.get_parameter("max_tracking_age_sec").value)
        )
        if not tracking_data_is_fresh(
            tracking_age, float(self.get_parameter("max_tracking_age_sec").value)
        ):
            transforms, buttons = {}, {}
        raw = transforms.get("r")
        a_pressed = bool(buttons.get("A", False))
        raw_b_pressed = bool(buttons.get("B", False))
        now = time.monotonic()
        if raw_b_pressed:
            self.b_release_since = None
            b_pressed = True
        elif self.previous_b_pressed:
            if self.b_release_since is None:
                self.b_release_since = now
            b_pressed = (
                now - self.b_release_since
                < float(self.get_parameter("b_release_debounce_sec").value)
            )
        else:
            b_pressed = False
        if self.previous_b_pressed and not b_pressed and self.adapter.allow_hardware:
            measured_q = self.adapter.read_joint_positions_rad()
            if measured_q is not None:
                # Release is an immediate stop-at-current-position event. Do
                # not keep chasing the last stale IK target.
                self.last_valid_q = np.asarray(measured_q, dtype=float).copy()
                self.ik.last_q = self.last_valid_q.copy()
                self.adapter.set_command_reference(self.last_valid_q)
                self.get_logger().info(
                    "B released: holding measured current joint position",
                    throttle_duration_sec=1.0,
                )
        self.previous_b_pressed = b_pressed
        raw_current = quest_to_robot_transform(raw) if raw is not None else None
        current = raw_current
        if raw_current is not None and self.last_input_pose is not None:
            discontinuity = pose_jump_exceeds(
                self.last_input_pose,
                raw_current,
                float(self.get_parameter("max_input_discontinuity_m").value),
                float(self.get_parameter("max_input_discontinuity_rad").value),
            )
            if discontinuity and b_pressed:
                self.input_jump_latched = True
                current = self.last_input_pose.copy()
                self.get_logger().warning(
                    "VR tracking jump rejected; release B to re-center",
                    throttle_duration_sec=1.0,
                )
            elif self.input_jump_latched and b_pressed:
                current = self.last_input_pose.copy()
            elif not b_pressed:
                self.input_jump_latched = False
                current = raw_current
            else:
                current = limit_input_pose_jump(
                    self.last_input_pose,
                    raw_current,
                    float(self.get_parameter("max_input_translation_step_m").value),
                    float(self.get_parameter("max_input_rotation_step_rad").value),
                )
        if current is not None:
            current = apply_wrist_pivot(
                current, self.get_parameter("wrist_pivot_offset_m").value
            )
            self.pose_history.append(current)
            current = average_pose_window(self.pose_history)
        require_b = bool(self.get_parameter("require_b_button").value)
        state_ready = hardware_control_ready(
            self.adapter.allow_hardware, self.measured_state_synced
        )
        if b_pressed:
            if self.b_pressed_since is None:
                self.b_pressed_since = now
            b_gate_active = (
                state_ready
                and (not require_b
                or now - self.b_pressed_since
                >= float(self.get_parameter("b_debounce_sec").value))
            )
        else:
            self.b_pressed_since = None
            b_gate_active = state_ready and not require_b
        if should_reset_vr_reference(self.previous_b_gate_active, b_gate_active):
            self.anchor_pending = not a_pressed
            self.anchor_started = now
            self.anchor_history.clear()
            if a_pressed and current is not None:
                self.base = current.copy()
                self.control_origin_pose = (
                    self.ik.forward_ee_pose(self.last_valid_q)
                    if self.last_valid_q is not None else self.startup_pose.copy()
                )
                self.pose_history.clear()
                self.pose_history.append(current)
                self.filtered_target = None
        if not b_gate_active:
            self.anchor_pending = False
            self.anchor_started = None
            self.anchor_history.clear()
        elif self.anchor_pending and current is not None:
            self.anchor_history.append(current.copy())
            elapsed = now - (self.anchor_started or now)
            stable = pose_window_is_stable(
                list(self.anchor_history),
                float(self.get_parameter("b_anchor_max_translation_m").value),
                float(self.get_parameter("b_anchor_max_rotation_rad").value),
            )
            if (
                elapsed >= float(self.get_parameter("b_anchor_settle_sec").value)
                and stable
            ):
                anchor = average_pose_window(list(self.anchor_history))
                self.base = anchor.copy()
                self.control_origin_pose = (
                    self.ik.forward_ee_pose(self.last_valid_q)
                    if self.last_valid_q is not None else self.startup_pose.copy()
                )
                self.pose_history.clear()
                self.pose_history.append(anchor)
                self.filtered_target = None
                self.anchor_pending = False
                self.anchor_started = None
        self.previous_b_gate_active = b_gate_active
        if current is not None:
            self.last_input_pose = current.copy()
        vr_active = b_gate_active and not self.anchor_pending
        q = None
        gripper = self.last_gripper
        clipped = False
        target_position = None
        base_delta_m = None
        target_delta_m = None
        ik_status = "not_attempted"
        input_xyz = None if raw_current is None else raw_current[:3, 3].copy()
        input_delta = None
        if input_xyz is not None and self.last_input_xyz is not None:
            input_delta = float(np.linalg.norm(input_xyz - self.last_input_xyz))
        if input_xyz is not None:
            self.last_input_xyz = input_xyz.copy()

        if vr_active and current is not None:
            if self.base is None:
                self.base = current.copy()
            target = relative_pose(self.base, current, self.control_origin_pose)
            base_delta_m = float(np.linalg.norm(current[:3, 3] - self.base[:3, 3]))
            target = scale_target_translation(
                target,
                float(self.get_parameter("translation_scale").value),
                origin=self.control_origin_pose[:3, 3],
            )
            target = scale_target_rotation(
                target, float(self.get_parameter("rotation_scale").value)
            )
            wall_reference = self.filtered_target if self.filtered_target is not None else target
            wall_min = np.asarray(self.get_parameter("workspace_min").value, dtype=float)
            wall_max = np.asarray(self.get_parameter("workspace_max").value, dtype=float)
            margin = float(self.get_parameter("workspace_margin_m").value)
            target, clipped = limit_target_translation_with_wall(
                wall_reference, target, wall_min + margin, wall_max - margin
            )
            reference_ee = (
                self.ik.forward_ee_pose(self.last_valid_q)
                if self.last_valid_q is not None else self.startup_pose
            )
            target, reach_clipped = limit_pose_error(
                reference_ee,
                target,
                float(self.get_parameter("reach_limit_m").value),
                float(self.get_parameter("reach_limit_rad").value),
            )
            clipped = clipped or reach_clipped
            if clipped:
                self.get_logger().warning(
                    "VR target clipped to Piper workspace", throttle_duration_sec=1.0
                )
            if self.filtered_target is None:
                self.filtered_target = target
            else:
                target = apply_pose_deadband(
                    self.filtered_target,
                    target,
                    float(self.get_parameter("pose_deadband_m").value),
                    float(self.get_parameter("pose_deadband_rad").value),
                )
                self.filtered_target = adaptive_pose_step(
                    self.filtered_target,
                    target,
                    response_alpha=float(self.get_parameter("pose_smoothing_alpha").value),
                    rotation_response_alpha=float(self.get_parameter("rotation_smoothing_alpha").value),
                    fast_translation_response_alpha=float(self.get_parameter("fast_translation_response_alpha").value),
                    fast_rotation_response_alpha=float(self.get_parameter("fast_rotation_response_alpha").value),
                    max_translation_step=float(self.get_parameter("max_cartesian_step_m").value),
                    max_rotation_step=float(self.get_parameter("max_rotation_step_rad").value),
                    fast_translation_threshold=float(self.get_parameter("adaptive_translation_threshold_m").value),
                    fast_rotation_threshold=float(self.get_parameter("adaptive_rotation_threshold_rad").value),
                )
            target_position = self.filtered_target[:3, 3].copy()
            target_delta_m = float(
                np.linalg.norm(target_position - self.startup_pose[:3, 3])
            )
            q, status = self.ik.solve_with_status(self.filtered_target)
            ik_status = status
            if status != "ok":
                self.get_logger().warning(
                    f"IK status: {status}", throttle_duration_sec=1.0
                )
                q = None
            else:
                self.last_valid_q = np.asarray(q, dtype=float).copy()
                values = buttons.get("rightTrig", [0.0])
                gripper = float(values[0]) * 0.07
                self.last_gripper = gripper

        self.home_latched = update_home_latch(
            a_pressed,
            b_pressed,
            self.home_q is not None,
            self.home_latched,
            bool(self.get_parameter("home_requires_b").value),
        )
        mode = select_command_mode(
            a_pressed,
            b_gate_active,
            q is not None,
            self.last_valid_q is not None,
            self.home_q is not None,
            bool(self.get_parameter("hold_when_b_released").value),
            bool(self.get_parameter("home_requires_b").value),
            self.home_latched,
        )
        if mode == "return_home":
            self.last_valid_q = self.home_q.copy()
            q = self.home_q.copy()
            gripper = 0.0
            self.get_logger().info("Returning to configured safe home pose", throttle_duration_sec=1.0)
        elif mode == "hold":
            q = self.last_valid_q
        elif mode == "no_command":
            return

        if target_position is not None:
            target_text = ",".join(f"{float(value):.3f}" for value in target_position)
        else:
            target_text = "none"
        delta_text = "none" if input_delta is None else f"{input_delta:.4f}"
        base_delta_text = "none" if base_delta_m is None else f"{base_delta_m:.4f}"
        target_delta_text = "none" if target_delta_m is None else f"{target_delta_m:.4f}"
        feedback_text = "none"
        tracking_age_text = "none" if tracking_age is None else f"{tracking_age * 1000.0:.0f}"
        enable_text = "none"
        if self.adapter.allow_hardware:
            enable_status = self.adapter.read_enable_status()
            if enable_status is not None:
                enable_text = "".join("1" if enabled else "0" for enabled in enable_status)
            feedback = self.adapter.read_joint_positions_rad()
            command = self.adapter.last_command_rad()
            if feedback is not None and command is not None:
                feedback_text = f"{max(abs(a - b) for a, b in zip(feedback, command)):.4f}"
        self.get_logger().info(
            f"VR_DIAG mode={mode} target_xyz=[{target_text}] "
            f"tracking={int(tracking_fresh)} controller_pose={right_pose_present} "
            f"tracking_age_ms={tracking_age_text} "
            f"clipped={clipped} ik={ik_status} "
            f"buttons=A:{int(a_pressed)} B:{int(b_pressed)} "
            f"b_gate={int(b_gate_active)} "
            f"input_delta_m={delta_text} "
            f"base_delta_m={base_delta_text} "
            f"target_delta_m={target_delta_text} "
            f"enable={enable_text} feedback_err_rad={feedback_text}",
            throttle_duration_sec=1.0,
        )

        if q is None:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [f"joint{i}" for i in range(1, 8)]
        msg.position = [float(x) for x in q] + [gripper]
        self.pub.publish(msg)
        if self.adapter.allow_hardware:
            self.adapter.send_joint_command(q, gripper)

    def destroy_node(self):
        self.reader.stop()
        self.adapter.close(bool(self.get_parameter("disable_on_exit").value))
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = QuestSinglePiperNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
