#!/usr/bin/env bash
set -euo pipefail
PATTERN="ros2 launch create_bringup create_1.launch"
TIMEOUT=${TIMEOUT:-10}
if ! pgrep -f "$PATTERN" >/dev/null 2>&1; then
  echo "[foot/shutdown] No matching processes found for pattern: $PATTERN"
  exit 0
fi

echo "[foot/shutdown] Sending SIGTERM to processes for: $PATTERN"
pkill -TERM -f "$PATTERN" || true

for ((i=0; i<TIMEOUT; i++)); do
  sleep 1
  if ! pgrep -f "$PATTERN" >/dev/null 2>&1; then
    echo "[foot/shutdown] All processes stopped"
    exit 0
  fi
done

echo "[foot/shutdown] Forcing SIGKILL for remaining processes"
pkill -KILL -f "$PATTERN" || true
