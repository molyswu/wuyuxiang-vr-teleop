#!/bin/bash

# =============================================================================
# Piper Master/Slave Dataset Recording Script (Multi-Task Support)
# =============================================================================
# Both arms share the same CAN bus (can0 by default).
# Two cameras: wrist (arm-mounted) + up (global overhead, from stereo depth cam).
#
# Prerequisites:
#   1. Activate CAN: bash $CONDA_PREFIX/lib/python3.10/site-packages/piper_sdk/can_activate.sh
#   2. Set master/slave modes: bash piper_scripts/setup_piper_master_slave.sh
#   3. Power on the SLAVE arm first, then the MASTER arm.
#
# Usage (方案一：多 Task 共享同一数据集):
#   # 第 1 次录制：创建新数据集（加 --new；若同名目录已存在会先备份）
#   bash piper_scripts/record_piper.sh press_elevator_button "Press 1st floor button" 5 --new
#
#   # 第 2 次录制：换 task，自动追加到同一数据集
#   bash piper_scripts/record_piper.sh press_elevator_button "Press 2nd floor button" 5
#
#   # 第 3 次录制：继续换 task，自动追加
#   bash piper_scripts/record_piper.sh press_elevator_button "Press open door button" 5
#
# Parameters:
#   $1: dataset_name   - 数据集名称（固定不变，决定存储路径和 repo_id）
#   $2: task_name      - 本次录制的 task 描述（可含空格，会自动处理）
#   $3: num_episodes   - 本次录制多少个 episode（默认 5）
#   $4: --new          - 可选，创建新数据集（若目录已存在会先移到带时间戳的备份目录）
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LEROBOT_ROOT="${LEROBOT_ROOT:-$(cd "$SCRIPT_DIR/../lerobot" && pwd)}"
CAN_NAME="${CAN_NAME:-can0}"
UP_CAMERA="${UP_CAMERA:-/dev/video4}"
WRIST_CAMERA="${WRIST_CAMERA:-/dev/video10}"
if [ "${PIPER_HARDWARE_CONFIRM:-}" != "YES" ]; then
    echo "Refusing to access Piper hardware. Set PIPER_HARDWARE_CONFIRM=YES after checking CAN, E-stop, and arm state." >&2
    exit 2
fi

CONDA_ENV="${CONDA_ENV:-vt}"
CONDA_SH="${CONDA_SH:-$HOME/miniconda3/etc/profile.d/conda.sh}"
DATASET_ROOT_BASE="${DATASET_ROOT_BASE:-/home/mc509/Workspace/VLA/quest/datasets}"
export PYTHONNOUSERSITE=1

if [ -f "$CONDA_SH" ]; then
    # Make conda available in non-interactive SSH shells.
    # shellcheck disable=SC1090
    source "$CONDA_SH"
fi

if ! command -v conda >/dev/null 2>&1; then
    echo "ERROR: conda not found. Set CONDA_SH to your conda.sh path." >&2
    exit 1
fi

# 解析参数
DATASET_NAME="${1:-press_elevator_button}"
TASK_NAME="${2:-piper_task}"
NUM_EPISODES="${3:-5}"

# 检测 --new flag（默认自动 resume 追加）
RESUME=true
if [ "$4" == "--new" ] || [ "$4" == "-n" ]; then
    RESUME=false
fi

# HuggingFace repo_id 不能包含空格，用下划线替换
SAFE_DATASET_NAME="${DATASET_NAME// /_}"

HF_USER="${HF_USER:-$USER}"
REPO_ID="${HF_USER}/${SAFE_DATASET_NAME}"
DATASET_ROOT="${DATASET_ROOT_BASE}/${SAFE_DATASET_NAME}"

echo "========================================"
echo "Piper Dataset Recording (Single CAN)"
echo "========================================"
echo "Dataset:     $DATASET_NAME"
echo "Task:        $TASK_NAME"
echo "Episodes:    $NUM_EPISODES"
echo "Repo ID:     $REPO_ID"
echo "Root:        $DATASET_ROOT"
echo "Resume:      $RESUME"
echo "Conda env:   $CONDA_ENV"
echo "Shared CAN:  $CAN_NAME"
echo "Cameras:     wrist ($WRIST_CAMERA) + up ($UP_CAMERA) via OpenCV V4L2"
echo ""

if [ "${PIPER_DRY_RUN:-0}" = "1" ]; then
    echo "Dry run: no dataset directory changes and no LeRobot process started."
    exit 0
fi

if [ "$RESUME" = true ]; then
    echo ">>> Resuming recording on existing dataset..."
else
    echo ">>> Creating new dataset..."
    if [ -e "$DATASET_ROOT" ]; then
        BACKUP_ROOT="${DATASET_ROOT}_backup_$(date +%Y%m%d_%H%M%S)"
        echo ">>> Existing dataset found. Moving it to: $BACKUP_ROOT"
        mv "$DATASET_ROOT" "$BACKUP_ROOT"
    fi
fi
echo ""

PYTHONPATH="$LEROBOT_ROOT/src${PYTHONPATH:+:$PYTHONPATH}" conda run -n "$CONDA_ENV" --no-capture-output lerobot-record \
    --robot.type=piper_follower \
    --robot.can_name="$CAN_NAME" \
    --robot.id=piper_slave \
    --robot.cameras="{
        up:    {type: opencv, index_or_path: \"$UP_CAMERA\", width: 640, height: 480, fps: 30, backend: 200},
        wrist: {type: opencv, index_or_path: \"$WRIST_CAMERA\", width: 640, height: 480, fps: 30, backend: 200}
    }" \
    --teleop.type=piper_leader \
    --teleop.can_name="$CAN_NAME" \
    --teleop.id=piper_master \
    --dataset.repo_id="$REPO_ID" \
    --dataset.root="$DATASET_ROOT" \
    --dataset.num_episodes="$NUM_EPISODES" \
    --dataset.single_task="$TASK_NAME" \
    --dataset.fps=30 \
    --dataset.episode_time_s=60 \
    --dataset.reset_time_s=50 \
    --dataset.push_to_hub=false \
    --robot.initial_pose='{
        joint_1.pos: 19.95,
        joint_2.pos: 11.92,
        joint_3.pos: -54.56,
        joint_4.pos: 3.44,
        joint_5.pos: 38.40,
        joint_6.pos: -5.15,
        gripper.pos: 0.01
    }' \
    --dataset.video=true \
    --display_data=true \
    --resume="$RESUME"

echo ""
echo "Recording complete. Dataset saved to: $DATASET_ROOT"
if [ "$RESUME" = true ]; then
    echo ""
    echo "To start a new dataset (existing dataset will be backed up first), run:"
    echo "  bash piper_scripts/record_piper.sh \"$DATASET_NAME\" \"Your task\" $NUM_EPISODES --new"
fi
