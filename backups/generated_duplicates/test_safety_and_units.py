import math
import numpy as np

import unittest

from quest_teleop_ros2.piper_sdk_adapter import (
    JOINT_RAD_TO_SDK,
    PiperSdkAdapter,
    radians_to_sdk_joints,
)
from quest_teleop_ros2.pose_math import relative_pose, xyzrpy_matrix


class SafetyAndUnitsTest(unittest.TestCase):
    def test_radians_to_sdk_joints_uses_piper_joint_scale(self):
        self.assertEqual(
            radians_to_sdk_joints([math.pi, -math.pi, 0, 0, 0, 0]),
            [
                round(math.pi * JOINT_RAD_TO_SDK),
                round(-math.pi * JOINT_RAD_TO_SDK),
                0,
                0,
                0,
                0,
            ],
        )


    def test_radians_to_sdk_joints_rejects_wrong_joint_count(self):
        with self.assertRaisesRegex(ValueError, "six arm joints"):
            radians_to_sdk_joints([0, 0, 0])


    def test_adapter_does_not_enable_or_send_without_explicit_hardware_enable(self):
        adapter = PiperSdkAdapter(allow_hardware=False)

        self.assertFalse(adapter.hardware_enabled)
        self.assertFalse(adapter.connect())
        self.assertFalse(adapter.send_joint_command([0, 0, 0, 0, 0, 0], 0.0))

    def test_relative_pose_preserves_configured_robot_origin(self):
        base = xyzrpy_matrix(0.1, 0.2, 0.3, 0.0, 0.0, 0.0)
        self.assertTrue(np.allclose(relative_pose(base, base), xyzrpy_matrix(0.19, 0.0, 0.2, 0, 0, 0)))


if __name__ == "__main__":
    unittest.main()
