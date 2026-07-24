#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
QUEST_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# The ROS setup script reads this variable while nounset is active.
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES-}"
set +u
source "$QUEST_ROOT/quest_ros2_env.sh"
set -u
export PYTHONPATH="$QUEST_ROOT/lerobot/src${PYTHONPATH:+:$PYTHONPATH}"

exec python "$SCRIPT_DIR/vr_record_piper.py" "$@"
