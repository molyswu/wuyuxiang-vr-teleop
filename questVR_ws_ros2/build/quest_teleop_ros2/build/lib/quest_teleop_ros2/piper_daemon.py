from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .piper_sdk_adapter import PiperSdkAdapter


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
        self.timer = self.create_timer(0.02, self.keep_enabled)
        self.get_logger().warning(
            "Persistent Piper daemon active; it will stay enabled until manually stopped"
        )

    def command_callback(self, message: JointState):
        if len(message.position) < 6:
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
        self.adapter.send_joint_command(self.command_q, self.last_gripper)

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
