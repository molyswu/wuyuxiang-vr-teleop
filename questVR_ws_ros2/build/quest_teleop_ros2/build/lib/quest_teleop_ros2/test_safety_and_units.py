import math
import numpy as np

import unittest


class _FakePiper:
    def __init__(self):
        self.disable_calls = 0
        self.disconnect_calls = 0
        self.joint_calls = 0

    def DisablePiper(self):
        self.disable_calls += 1

    def DisconnectPort(self):
        self.disconnect_calls += 1

    def MotionCtrl_2(self, *args):
        pass

    def JointCtrl(self, *args):
        self.joint_calls += 1

    def GripperCtrl(self, *args):
        pass

from quest_teleop_ros2.piper_sdk_adapter import (
    JOINT_RAD_TO_SDK,
    PiperSdkAdapter,
    radians_to_sdk_joints,
)
from quest_teleop_ros2.control_state import select_command_mode
from quest_teleop_ros2.pose_math import (
    limit_target_translation,
    relative_pose,
    scale_target_rotation,
    scale_target_translation,
    smooth_pose,
    xyzrpy_matrix,
)


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

    def test_adapter_defaults_to_conservative_motion_limits(self):
        adapter = PiperSdkAdapter(allow_hardware=False)
        self.assertEqual(adapter.speed_rate, 10)
        self.assertAlmostEqual(adapter.max_joint_step_rad, 0.01)

    def test_adapter_reference_is_inert_without_hardware(self):
        adapter = PiperSdkAdapter(allow_hardware=False)
        adapter.set_command_reference([0, 0, 0, 0, 0, 0])
        self.assertFalse(adapter.send_joint_command([0.1, 0, 0, 0, 0, 0], 0.0))

    def test_normal_close_does_not_disable_motors(self):
        adapter = PiperSdkAdapter(allow_hardware=True)
        fake = _FakePiper()
        adapter._piper = fake
        adapter._connected = True
        adapter.hardware_enabled = True
        adapter.set_command_reference([0, 0, 0, 0, 0, 0])
        adapter.close()
        self.assertEqual(fake.disable_calls, 0)
        self.assertEqual(fake.joint_calls, 1)
        self.assertEqual(fake.disconnect_calls, 1)

    def test_explicit_close_can_disable_motors(self):
        adapter = PiperSdkAdapter(allow_hardware=True)
        fake = _FakePiper()
        adapter._piper = fake
        adapter._connected = True
        adapter.hardware_enabled = True
        adapter.set_command_reference([0, 0, 0, 0, 0, 0])
        adapter.close(disable=True)
        self.assertEqual(fake.disable_calls, 1)

    def test_relative_pose_preserves_configured_robot_origin(self):
        base = xyzrpy_matrix(0.1, 0.2, 0.3, 0.0, 0.0, 0.0)
        self.assertTrue(np.allclose(relative_pose(base, base), xyzrpy_matrix(0.19, 0.0, 0.2, 0, 0, 0)))

    def test_target_translation_is_scaled_about_robot_origin_and_clamped(self):
        target = xyzrpy_matrix(0.80, -0.8, 1.0, 0.1, 0.2, 0.3)
        limited, clipped = limit_target_translation(
            target, scale=0.5,
            minimum=(0.05, -0.2, 0.05), maximum=(0.4, 0.2, 0.5),
        )
        self.assertTrue(clipped)
        self.assertTrue(np.allclose(limited[:3, 3], [0.4, -0.2, 0.5]))
        self.assertTrue(np.allclose(limited[:3, :3], target[:3, :3]))

    def test_translation_scaling_is_about_piper_origin(self):
        target = xyzrpy_matrix(0.39, 0.0, 0.30, 0, 0, 0)
        scaled = scale_target_translation(target, 0.5)
        self.assertTrue(np.allclose(scaled[:3, 3], [0.29, 0.0, 0.25]))

    def test_rotation_scaling_preserves_translation(self):
        target = xyzrpy_matrix(0.19, 0.0, 0.2, 0, 0, math.pi / 2)
        scaled = scale_target_rotation(target, 0.5)
        expected = xyzrpy_matrix(0.19, 0.0, 0.2, 0, 0, math.pi / 4)
        self.assertTrue(np.allclose(scaled[:3, 3], target[:3, 3]))
        self.assertTrue(np.allclose(scaled[:3, :3], expected[:3, :3], atol=1e-6))

    def test_pose_smoothing_interpolates_translation(self):
        old = xyzrpy_matrix(0.19, 0.0, 0.2, 0, 0, 0)
        new = xyzrpy_matrix(0.39, 0.0, 0.2, 0, 0, 0)
        filtered = smooth_pose(old, new, 0.25)
        self.assertTrue(np.allclose(filtered[:3, 3], [0.24, 0.0, 0.2]))

    def test_control_state_holds_on_b_release(self):
        self.assertEqual(
            select_command_mode(False, False, False, True, True, True), "hold"
        )

    def test_control_state_requires_b_for_startup_return(self):
        self.assertEqual(
            select_command_mode(True, True, True, True, True, True), "return_home"
        )
        self.assertEqual(
            select_command_mode(True, False, True, True, True, True), "hold"
        )

    def test_control_state_uses_vr_only_when_b_is_pressed(self):
        self.assertEqual(
            select_command_mode(False, True, True, True, True, True), "vr_control"
        )
        self.assertEqual(
            select_command_mode(False, False, False, False, True, True), "no_command"
        )

    def test_home_target_is_configured_safe_pose(self):
        from quest_teleop_ros2.safe_home_config import SAFE_HOME_Q
        self.assertEqual(len(SAFE_HOME_Q), 6)
        self.assertTrue(all(np.isfinite(SAFE_HOME_Q)))
        self.assertNotEqual(tuple(SAFE_HOME_Q), (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
