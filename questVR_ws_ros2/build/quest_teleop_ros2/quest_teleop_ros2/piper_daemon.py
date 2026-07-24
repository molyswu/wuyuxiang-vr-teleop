from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .piper_sdk_adapter import PiperSdkAdapter


def command_period_s(rate_hz: float) -> float:
    rate = float(rate_hz)
    if not math.isfinite(rate) or rate <= 0:
        raise ValueError("command_rate_hz must be positive and finite")
    return 1.0 / rate


class PiperDaemon(Node):
    """Persistent Piper SDK owner for VR clients.

    The daemon intentionally keeps the SDK/CAN connection and motor enable
    state after a VR client exits. Stop it explicitly to disable the arm.
    """

    def __init__(self):
        super().__init__("piper_daemon")
        self.declare_parameter("can_name", "can0")
        self.declare_parameter("speed_rate", 10)
        self.declare_parameter("max_joint_step_rad", 0.006)
        self.declare_parameter("command_rate_hz", 30)
        self.command_rate_hz = float(self.get_parameter("command_rate_hz").value)
        self.adapter = PiperSdkAdapter(
            self.get_parameter("can_name").value,
            True,
            int(self.get_parameter("speed_rate").value),
            float(self.get_parameter("max_joint_step_rad").value),
        )
        if not self.adapter.connect():
            self.adapter.close(disable=True)
            raise RuntimeError("Piper SDK could not connect/enable the arm")
        measured = self.adapter.read_joint_positions_rad()
        if measured is None:
            self.adapter.close(disable=True)
            raise RuntimeError("Could not read Piper joint state")
        self.measured_q = list(measured)
        self.command_q = list(measured)
        self.last_gripper = 0.0
        self.adapter.set_command_reference(self.command_q)
        self.measured_pub = self.create_publisher(
            JointState, "piper_measured_joint_state", 10
        )
        self.subscription = self.create_subscription(
            JointState, "joint_command", self.command_callback, 10
        )
        self.timer = self.create_timer(
            command_period_s(self.command_rate_hz), self.keep_enabled
        )
        self._fault_reported = False
        self.get_logger().warning(
            f"Persistent Piper daemon active at {self.command_rate_hz:.1f} Hz; "
            "it will stay enabled until manually stopped"
        )

    def command_callback(self, message: JointState):
        if self.adapter.send_faulted or len(message.position) < 6:
            return
        self.command_q = [float(value) for value in message.position[:6]]
        if len(message.position) >= 7:
            self.last_gripper = float(message.position[6])

    def keep_enabled(self):
        measured = self.adapter.read_joint_positions_rad()
        if measured is not None:
            self.measured_q = list(measured)
            state = JointState()
            state.name = [f"joint{i}" for i in range(1, 7)]
            state.position = list(measured)
            self.measured_pub.publish(state)
        if self.adapter.send_faulted:
            if not self._fault_reported:
                self._fault_reported = True
                self.get_logger().error(
                    f"Piper send fault latched; stopped sending commands: "
                    f"{self.adapter.last_fault}"
                )
        ok = self.adapter.send_joint_command(self.command_q, self.last_gripper)
        if not ok and self.adapter.send_faulted and not self._fault_reported:
            self._fault_reported = True
            self.get_logger().error(
                f"Piper send fault latched; stopped sending commands: "
                f"{self.adapter.last_fault}"
            )
    def destroy_node(self):
        # Explicit daemon shutdown is the manual stop path and disables Piper.
        self.adapter.close(disable=True)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PiperDaemon()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
