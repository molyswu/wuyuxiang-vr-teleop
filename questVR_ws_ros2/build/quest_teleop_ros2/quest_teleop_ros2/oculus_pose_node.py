from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from .oculus_reader import OculusReader


def xyzrpy_to_matrix(x, y, z, roll, pitch, yaw):
    matrix = np.eye(4)
    a, b = math.cos(yaw), math.sin(yaw)
    c, d = math.cos(pitch), math.sin(pitch)
    e, f = math.cos(roll), math.sin(roll)
    matrix[:3, :3] = [
        [a * c, a * d * f - b * e, b * f + a * d * e],
        [b * c, a * e + b * d * f, b * d * e - a * f],
        [-d, c * f, c * e],
    ]
    matrix[:3, 3] = [x, y, z]
    return matrix


def adjustment_matrix(transform):
    transform = np.asarray(transform, dtype=float)
    if transform.shape != (4, 4):
        raise ValueError("transform must be a 4x4 matrix")
    axis_adjust = np.array([[0, 0, -1, 0], [-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]])
    rotation_adjust = xyzrpy_to_matrix(0, 0, 0, -math.pi, 0, -math.pi / 2)
    return axis_adjust @ transform @ rotation_adjust


def rotation_matrix_to_quaternion(matrix):
    trace = float(np.trace(matrix[:3, :3]))
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        w = 0.25 * s
        x = (matrix[2, 1] - matrix[1, 2]) / s
        y = (matrix[0, 2] - matrix[2, 0]) / s
        z = (matrix[1, 0] - matrix[0, 1]) / s
    else:
        values = np.diag(matrix[:3, :3])
        i = int(np.argmax(values))
        if i == 0:
            s = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2
            x, y, z, w = 0.25 * s, (matrix[0, 1] + matrix[1, 0]) / s, (matrix[0, 2] + matrix[2, 0]) / s, (matrix[2, 1] - matrix[1, 2]) / s
        elif i == 1:
            s = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2
            x, y, z, w = (matrix[0, 1] + matrix[1, 0]) / s, 0.25 * s, (matrix[1, 2] + matrix[2, 1]) / s, (matrix[0, 2] - matrix[2, 0]) / s
        else:
            s = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2
            x, y, z, w = (matrix[0, 2] + matrix[2, 0]) / s, (matrix[1, 2] + matrix[2, 1]) / s, 0.25 * s, (matrix[1, 0] - matrix[0, 1]) / s
    return x, y, z, w


class OculusPoseNode(Node):
    def __init__(self):
        super().__init__("oculus_pose_node")
        self.right_pub = self.create_publisher(PoseStamped, "right_handle_pose", 1)
        self.left_pub = self.create_publisher(PoseStamped, "left_handle_pose", 1)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.reader = OculusReader()
        self.timer = self.create_timer(0.01, self.publish_once)

    def publish_once(self):
        transformations, _ = self.reader.get_transformations_and_buttons()
        stamp = self.get_clock().now().to_msg()
        for key, publisher, child in (("r", self.right_pub, "right_controller"), ("l", self.left_pub, "left_controller")):
            if key not in transformations:
                continue
            matrix = adjustment_matrix(transformations[key])
            x, y, z = matrix[:3, 3]
            qx, qy, qz, qw = rotation_matrix_to_quaternion(matrix)
            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = "vr_device"
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = float(x), float(y), float(z)
            pose.pose.orientation.x, pose.pose.orientation.y = float(qx), float(qy)
            pose.pose.orientation.z, pose.pose.orientation.w = float(qz), float(qw)
            publisher.publish(pose)
            transform = TransformStamped()
            transform.header = pose.header
            transform.child_frame_id = child
            transform.transform.translation.x = float(x)
            transform.transform.translation.y = float(y)
            transform.transform.translation.z = float(z)
            transform.transform.rotation = pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)

    def destroy_node(self):
        self.reader.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OculusPoseNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
