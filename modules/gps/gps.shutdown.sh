#!/bin/bash
set -euo pipefail
PATTERN="ros2 launch ublox_gps ublox_gps.launch.py"
TIMEOUT=${TIMEOUT:-10}
if ! pgrep -f "$PATTERN" >/dev/null 2>&1; then
  echo "[gps/shutdown] No matching processes found for pattern: $PATTERN"
  exit 0
fi

echo "[gps/shutdown] Sending SIGTERM to processes for: $PATTERN"
pkill -TERM -f "$PATTERN" || true

for ((i=0; i<TIMEOUT; i++)); do
  sleep 1
  if ! pgrep -f "$PATTERN" >/dev/null 2>&1; then
    echo "[gps/shutdown] All processes stopped"
    exit 0
  fi
done

echo "[gps/shutdown] Forcing SIGKILL for remaining processes"
pkill -KILL -f "$PATTERN" || true
