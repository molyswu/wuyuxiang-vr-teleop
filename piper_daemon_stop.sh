#!/usr/bin/env bash
set -Eeuo pipefail
PID_FILE="/tmp/quest_piper_daemon.pid"
if [[ ! -f "$PID_FILE" ]]; then
    echo "Piper daemon is not running"
    exit 0
fi
pid="$(cat "$PID_FILE")"
kill -INT "$pid" 2>/dev/null || true
rm -f "$PID_FILE"
echo "Piper daemon stopped and Piper disabled"
