#!/usr/bin/env bash
set -Eeuo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PID_FILE="/tmp/quest_piper_daemon.pid"
LOG_FILE="/tmp/quest_piper_daemon.log"

if [[ -f "$PID_FILE" ]] && kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
    echo "Piper daemon already running: $(cat "$PID_FILE")"
    exit 0
fi

if ! ip link show can0 >/dev/null 2>&1; then
    echo "ERROR: can0 does not exist" >&2
    exit 1
fi

can_state="$(ip -details link show can0 | awk '/can state/ {print $3; exit}')"
if [[ "$can_state" != "ERROR-ACTIVE" ]]; then
    echo "CAN state is ${can_state:-unknown}; resetting CAN..."
    sudo -v
    sudo ip link set can0 down
    sudo ip link set can0 up type can bitrate 1000000 sample-point 0.750
fi

cd "$SCRIPT_DIR/questVR_ws_ros2"
nohup bash -lc "source '$SCRIPT_DIR/quest_ros2_env.sh' >/dev/null && exec python -c 'from quest_teleop_ros2.piper_daemon import main; main()' --ros-args -p can_name:=can0 -p speed_rate:=10 -p max_joint_step_rad:=0.006 -p command_rate_hz:=50" \
    >"$LOG_FILE" 2>&1 &
echo $! >"$PID_FILE"
sleep 2
if ! kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
    echo "Piper daemon failed; see $LOG_FILE" >&2
    exit 1
fi
echo "Piper daemon running: $(cat "$PID_FILE")"
