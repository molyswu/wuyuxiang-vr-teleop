"""Small, explicitly opt-in adapter around the Piper SDK.

The adapter is deliberately inert by default. Importing or constructing it
must not open a CAN device or enable the arm.
"""

from __future__ import annotations

import math
import time
from typing import Iterable, Optional

JOINT_RAD_TO_SDK = 57295.7795
GRIPPER_METER_TO_SDK = 1_000_000.0


def radians_to_sdk_joints(joints: Iterable[float]) -> list[int]:
    values = list(joints)
    if len(values) != 6:
        raise ValueError("expected six arm joints in radians")
    return [round(value * JOINT_RAD_TO_SDK) for value in values]


class PiperSdkAdapter:
    """Opt-in Piper SDK adapter for a single arm.

    ``allow_hardware`` must be explicitly set to True by a future launch
    configuration before any CAN connection or motion call is possible.
    """

    def __init__(
        self,
        can_name: str = "can0",
        allow_hardware: bool = False,
        speed_rate: int = 10,
        max_joint_step_rad: float = 0.01,
    ):
        self.can_name = can_name
        self.allow_hardware = allow_hardware
        self.speed_rate = int(speed_rate)
        self.max_joint_step_rad = float(max_joint_step_rad)
        if not 1 <= self.speed_rate <= 100:
            raise ValueError("speed_rate must be between 1 and 100")
        if self.max_joint_step_rad <= 0:
            raise ValueError("max_joint_step_rad must be positive")
        self.hardware_enabled = False
        self._connected = False
        self._piper: Optional[object] = None
        self._last_command_rad: Optional[list[float]] = None
        self.send_faulted = False
        self.last_fault = None
        self._joint_mode_configured = False

    def connect(self) -> bool:
        if not self.allow_hardware:
            return False

        from piper_sdk import C_PiperInterface_V2

        self._piper = C_PiperInterface_V2(self.can_name)
        self._piper.ConnectPort()
        self._connected = True
        # EnablePiper() returns the state observed *before* it sends the
        # enable frame.  Wait for the subsequent feedback instead of treating
        self.send_faulted = False
        self.last_fault = None
        # that first return value as the final result.
        self._piper.EnablePiper()
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            if all(self._piper.GetArmEnableStatus()):
                self._configure_joint_mode()
                self.hardware_enabled = True
                return True
            time.sleep(0.05)
        self.hardware_enabled = False
        return self.hardware_enabled

    def _configure_joint_mode(self) -> None:
        """Configure V2 CAN joint position-velocity mode once per connection."""
        if self._piper is None:
            return
        # ModeCtrl is the documented V2 interface. Keep a compatibility
        # fallback for older installed SDK builds that only expose MotionCtrl_2.
        if hasattr(self._piper, "ModeCtrl"):
            self._piper.ModeCtrl(0x01, 0x01, self.speed_rate, 0x00)
        else:
            self._piper.MotionCtrl_2(0x01, 0x01, self.speed_rate, 0x00)
        self._joint_mode_configured = True

    def set_command_reference(self, joints_rad: Iterable[float]) -> None:
        values = list(joints_rad)
        if len(values) != 6:
            raise ValueError("expected six arm joints in radians")
        self._last_command_rad = [float(value) for value in values]

    def read_joint_positions_rad(self) -> Optional[list[float]]:
        if not self._connected or self._piper is None:
            return None
        state = self._piper.GetArmJointMsgs().joint_state
        raw = [state.joint_1, state.joint_2, state.joint_3,
               state.joint_4, state.joint_5, state.joint_6]
        return [math.radians(float(value) / 1000.0) for value in raw]

    def read_enable_status(self) -> Optional[list[bool]]:
        if not self._connected or self._piper is None:
            return None
        return [bool(value) for value in self._piper.GetArmEnableStatus()]

    def last_command_rad(self) -> Optional[list[float]]:
        if self._last_command_rad is None:
            return None
        return list(self._last_command_rad)

    def _latch_send_fault(self, reason: str) -> bool:
        self.send_faulted = True
        self.last_fault = str(reason)
        self.hardware_enabled = False
        return False

    def send_joint_command(self, joints_rad: Iterable[float], gripper_m: float) -> bool:
        if (
            not self.allow_hardware
            or not self._connected
            or not self.hardware_enabled
            or self.send_faulted
        ):
            return False
        target = [float(value) for value in joints_rad]
        if len(target) != 6:
            raise ValueError("expected six arm joints in radians")
        can_bus = getattr(self._piper, "GetCanBus", lambda: None)()
        if can_bus is not None:
            status = can_bus.is_can_bus_ok()
            if status != can_bus.CAN_STATUS.BUS_STATE_ACTIVE:
                return self._latch_send_fault(f"CAN bus not active: {status}")
        if self._last_command_rad is None:
            self._last_command_rad = target
        else:
            limited = []
            for previous, desired in zip(self._last_command_rad, target):
                delta = max(-self.max_joint_step_rad,
                            min(self.max_joint_step_rad, desired - previous))
                limited.append(previous + delta)
            self._last_command_rad = limited
        joints = radians_to_sdk_joints(self._last_command_rad)
        gripper = round(gripper_m * GRIPPER_METER_TO_SDK)
        try:
            self._piper.JointCtrl(*joints)
            self._piper.GripperCtrl(abs(gripper), 1000, 0x01, 0)
        except Exception as exc:
            return self._latch_send_fault(f"SDK send exception: {exc}")
        if can_bus is not None:
            status = can_bus.is_can_bus_ok()
            if status != can_bus.CAN_STATUS.BUS_STATE_ACTIVE:
                return self._latch_send_fault(f"CAN bus failed after send: {status}")
        if hasattr(self._piper, "GetArmEnableStatus"):
            try:
                enable_status = self._piper.GetArmEnableStatus()
            except Exception as exc:
                return self._latch_send_fault(f"enable status read failed: {exc}")
            if not all(enable_status):
                return self._latch_send_fault("Piper enable status dropped during send")
        return True

    def close(self, disable: bool = False) -> None:
        if self._piper is not None and self._connected:
            if self._last_command_rad is not None and self.hardware_enabled:
                self.send_joint_command(self._last_command_rad, 0.0)
            if disable:
                self._piper.DisablePiper()
            self._piper.DisconnectPort()
        self._connected = False
        self.hardware_enabled = False
        self._joint_mode_configured = False
        self._last_command_rad = None
