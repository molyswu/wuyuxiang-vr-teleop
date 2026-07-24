#!/usr/bin/env bash
set -Eeuo pipefail

# Real Quest VR -> Piper test.
# This script starts the already-tested ROS2 teleoperation path.
# It does not start LeRobot recording and does not modify the APK.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CAN_NAME="${CAN_NAME:-can0}"

echo "=== Quest VR / Piper real test ==="
echo "CAN: $CAN_NAME"
echo ""
echo "Before continuing:"
echo "  1. Piper is powered and the emergency stop is ready."
echo "  2. The Quest is connected by ADB and the old teleop APK is open."
echo "  3. Keep the arm area clear. Hold B to enable control."
echo ""

if ! ip link show "$CAN_NAME" >/dev/null 2>&1; then
    echo "ERROR: $CAN_NAME does not exist." >&2
    exit 1
fi

if command -v adb >/dev/null 2>&1; then
    if ! adb get-state >/dev/null 2>&1; then
        echo "WARNING: ADB has no connected Quest. The ROS2 node may not receive VR data." >&2
    else
        echo "ADB: Quest connected"
    fi
fi

read -r -p "确认机械臂周围安全？输入 YES 开始真实测试: " confirm
if [[ "$confirm" != "YES" ]]; then
    echo "已取消，没有启动机械臂。"
    exit 0
fi

cd "$SCRIPT_DIR"
"$SCRIPT_DIR/piper_daemon_start.sh"

# The VR node is a client only. The daemon owns the SDK/CAN connection and
# remains alive after this client exits.
"$SCRIPT_DIR/run_quest_single_piper.sh" \
    --ros-args \
    -p allow_hardware:=false \
    -p can_name:="$CAN_NAME" \
    -p speed_rate:=10 \
    -p max_joint_step_rad:=0.005 \
    -p translation_scale:=1.2 \
    -p rotation_scale:=0.5 \
    -p pose_smoothing_alpha:=0.20 \
    -p fast_translation_response_alpha:=0.30 \
    -p rotation_smoothing_alpha:=0.18 \
    -p fast_rotation_response_alpha:=0.26 \
    -p pose_filter_window:=3 \
    -p wrist_pivot_offset_m:="[0.0, 0.0, 0.0]" \
    -p max_tracking_age_sec:=0.12 \
    -p b_anchor_settle_sec:=0.16 \
    -p b_anchor_max_translation_m:=0.01 \
    -p b_anchor_max_rotation_rad:=0.08 \
    -p pose_deadband_m:=0.005 \
    -p pose_deadband_rad:=0.040 \
    -p max_input_translation_step_m:=0.03 \
    -p max_input_rotation_step_rad:=0.16 \
    -p adaptive_translation_threshold_m:=0.015 \
    -p adaptive_rotation_threshold_rad:=0.08 \
    -p max_rotation_step_rad:=0.16 \
    -p reach_limit_m:=0.15 \
    -p reach_limit_rad:=0.80 \
    -p workspace_margin_m:=0.02 \
    -p hold_when_b_released:=true \
    -p home_requires_b:=true \
    -p require_b_button:=true \
    "$@"
