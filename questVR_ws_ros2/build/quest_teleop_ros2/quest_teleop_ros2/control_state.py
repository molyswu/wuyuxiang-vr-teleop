from __future__ import annotations
import math

def tracking_data_is_fresh(age_sec, max_age_sec=0.12):
    """Return whether the latest APK tracking sample is recent enough."""
    if max_age_sec <= 0 or age_sec is None:
        return False
    try:
        age = float(age_sec)
    except (TypeError, ValueError):
        return False
    return math.isfinite(age) and 0.0 <= age <= float(max_age_sec)

def should_reset_vr_reference(previous_b_gate_active: bool, b_gate_active: bool) -> bool:
    return bool(b_gate_active and not previous_b_gate_active)


def update_home_latch(
    a_pressed: bool,
    b_pressed: bool,
    has_startup_q: bool,
    home_latched: bool,
    home_requires_b: bool = True,
) -> bool:
    """Update the safe-home latch without issuing any hardware command.

    A+B enters the latch. Releasing the buttons keeps the arm at home.
    B alone is the explicit resume action; A alone never resumes.
    """
    if has_startup_q and a_pressed and (b_pressed or not home_requires_b):
        return True
    if home_latched and b_pressed and not a_pressed:
        return False
    return home_latched

def select_command_mode(
    a_pressed: bool,
    b_pressed: bool,
    has_vr_target: bool,
    has_last_valid_q: bool,
    has_startup_q: bool,
    hold_when_b_released: bool = True,
    home_requires_b: bool = True,
    home_latched: bool = False,
) -> str:
    """Select a safe command source without touching hardware."""
    if home_latched or (a_pressed and (b_pressed or not home_requires_b) and has_startup_q):
        return "return_home"
    if b_pressed and has_vr_target:
        return "vr_control"
    if hold_when_b_released and has_last_valid_q:
        return "hold"
    return "no_command"
def hardware_control_ready(allow_hardware, measured_state_synced):
    """Allow client-only control only after the daemon state is synchronized."""
    return bool(allow_hardware or measured_state_synced)

