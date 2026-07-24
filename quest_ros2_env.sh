#!/usr/bin/env bash
set -e

QUEST_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/questVR_ws_ros2" && pwd)"
source /home/mc509/miniconda3/etc/profile.d/conda.sh
conda activate vt
source /opt/ros/humble/setup.bash
export PYTHONNOUSERSITE=1
export AMENT_PREFIX_PATH="$QUEST_WS/install${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
if [ -f "$QUEST_WS/install/local_setup.bash" ]; then
  source "$QUEST_WS/install/local_setup.bash"
fi
export QUEST_ROS2_WS="$QUEST_WS"
echo "ROS2 workspace ready: $QUEST_WS"
