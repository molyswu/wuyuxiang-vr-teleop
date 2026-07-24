#!/usr/bin/env bash
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/quest_ros2_env.sh"
exec python -c 'from quest_teleop_ros2.quest_single_piper_node import main; main()' "$@"
