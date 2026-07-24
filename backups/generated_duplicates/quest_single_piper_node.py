from __future__ import annotations

import time
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .oculus_reader import OculusReader
from .piper_ik import PiperIK
from .piper_sdk_adapter import PiperSdkAdapter
from .pose_math import quest_to_robot_transform, relative_pose


class QuestSinglePiperNode(Node):
    def __init__(self):
        super().__init__("quest_single_piper_node")
        self.declare_parameter("urdf_path", "")
        self.declare_parameter("can_name", "can0")
        self.declare_parameter("allow_hardware", False)
        self.declare_parameter("quest_ip", "")
        self.declare_parameter("require_b_button", True)
        urdf = self.get_parameter("urdf_path").value
        if not urdf:
            urdf = get_package_share_directory("quest_teleop_ros2") + "/urdf/piper_description.urdf"
        self.ik = PiperIK(urdf)
        self.adapter = PiperSdkAdapter(
            self.get_parameter("can_name").value,
            bool(self.get_parameter("allow_hardware").value),
        )
        if self.adapter.allow_hardware and not self.adapter.connect():
            raise RuntimeError("Piper SDK could not connect/enable the arm")
        ip = self.get_parameter("quest_ip").value or None
        self.reader = OculusReader(ip_address=ip)
        self.pub = self.create_publisher(JointState, "joint_command", 10)
        self.base = None
        self.timer = self.create_timer(0.02, self.tick)
        self.get_logger().warning("Piper hardware is disabled unless allow_hardware:=true")

    def tick(self):
        transforms, buttons = self.reader.get_transformations_and_buttons()
        raw = transforms.get("r")
        if raw is None:
            return
        current = quest_to_robot_transform(raw)
        if buttons.get("A", False):
            self.base = current.copy()
            self.get_logger().info("VR base pose reset")
            return
        if self.base is None:
            self.base = current.copy()
        if self.get_parameter("require_b_button").value and not buttons.get("B", False):
            return
        target = relative_pose(self.base, current)
        q = self.ik.solve(target)
        if q is None:
            self.get_logger().warning("IK rejected target", throttle_duration_sec=1.0)
            return
        gripper = float(buttons.get("rightTrig", [0.0])[0]) * 0.07
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [f"joint{i}" for i in range(1, 8)]
        msg.position = [float(x) for x in q] + [gripper]
        self.pub.publish(msg)
        if self.adapter.allow_hardware:
            self.adapter.send_joint_command(q, gripper)

    def destroy_node(self):
        self.reader.stop()
        self.adapter.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = QuestSinglePiperNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
