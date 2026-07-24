#!/bin/bash

# =============================================================================
# Piper Master/Slave Teleoperation Script (Single CAN)
# =============================================================================
# Both arms share the same CAN bus (can0 by default).
#
# Prerequisites:
#   1. Activate CAN: bash piper_scripts/can_activate.sh
#   2. Set one Piper as MASTER, the other as SLAVE (via hardware switch or SDK).
#   3. Power on the SLAVE arm first, then the MASTER arm.
#
# Usage:
#   bash piper_scripts/teleop_piper.sh
#
# With camera (uncomment the CAMERA block below and comment out the basic one):
#   bash piper_scripts/teleop_piper.sh
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LEROBOT_ROOT="${LEROBOT_ROOT:-$(cd "$SCRIPT_DIR/../lerobot" && pwd)}"
CONDA_ENV="${CONDA_ENV:-vt}"
CAN_NAME="${CAN_NAME:-can0}"
UP_CAMERA="${UP_CAMERA:-/dev/video4}"
WRIST_CAMERA="${WRIST_CAMERA:-/dev/video10}"
if [ "${PIPER_HARDWARE_CONFIRM:-}" != "YES" ]; then
    echo "Refusing to access Piper hardware. Set PIPER_HARDWARE_CONFIRM=YES after checking CAN, E-stop, and arm state." >&2
    exit 2
fi
source "${CONDA_SH:-$HOME/miniconda3/etc/profile.d/conda.sh}"
conda activate "$CONDA_ENV"
export PYTHONPATH="$LEROBOT_ROOT/src${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONNOUSERSITE=1

echo "Starting Piper teleoperation (single CAN master/slave)..."
echo "  Shared CAN: $CAN_NAME"
echo "  Make sure SLAVE arm is powered on BEFORE the MASTER arm."
echo "  Press 'q' to quit gracefully."
echo ""

if [ "${PIPER_DRY_RUN:-0}" = "1" ]; then
    echo "Dry run: no LeRobot process started and no hardware accessed."
    exit 0
fi

# ---------------------------------------------------------------------------
# Basic teleoperation (no camera)
# ---------------------------------------------------------------------------
# lerobot-teleoperate \
#     --robot.type=piper_follower \
#     --robot.can_name=can0 \
#     --teleop.type=piper_leader \
#     --teleop.can_name=can0 \
#     --fps=30

# ---------------------------------------------------------------------------
# Teleoperation with camera (uncomment below to use)
# ---------------------------------------------------------------------------
# Adjust index_or_path, width, height, fps to match your setup.
# Use 'v4l2-ctl --list-devices' to find the correct camera index.
# 
# ---------------------------------------------------------------------------
# RealSense via native SDK (type: intelrealsense)
# ---------------------------------------------------------------------------
# lerobot-teleoperate \
#     --robot.type=piper_follower \
#     --robot.can_name=can0 \
#     --robot.cameras='{
#         up:    {type: intelrealsense, serial_number_or_name: "243322073287", width: 640, height: 480, fps: 30},
#         wrist: {type: intelrealsense, serial_number_or_name: "254522074742", width: 640, height: 480, fps: 30}
#     }' \
#     --teleop.type=piper_leader \
#     --teleop.can_name=can0 \
#     --display_data=true \
#     --fps=30

# ---------------------------------------------------------------------------
# Legacy: RealSense via V4L2/OpenCV (type: opencv)
# ---------------------------------------------------------------------------
lerobot-teleoperate \
    --robot.type=piper_follower \
    --robot.can_name="$CAN_NAME" \
    --robot.cameras="{
        up: {type: opencv, index_or_path: \"$UP_CAMERA\", width: 640, height: 480, fps: 30, backend: 200},
        wrist: {type: opencv, index_or_path: \"$WRIST_CAMERA\", width: 640, height: 480, fps: 30, backend: 200}
    }" \
    --teleop.type=piper_leader \
    --teleop.can_name="$CAN_NAME" \
    --display_data=true \
    --fps=30
