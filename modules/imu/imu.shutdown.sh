#!/bin/bash
set -euo pipefail
PATTERN="ros2 launch mpu6050driver mpu6050driver_launch.py"
TIMEOUT=${TIMEOUT:-10}
if ! pgrep -f "$PATTERN" >/dev/null 2>&1; then
  echo "[imu/shutdown] No matching processes found for pattern: $PATTERN"
  exit 0
fi

echo "[imu/shutdown] Sending SIGTERM to processes for: $PATTERN"
pkill -TERM -f "$PATTERN" || true

for ((i=0; i<TIMEOUT; i++)); do
  sleep 1
  if ! pgrep -f "$PATTERN" >/dev/null 2>&1; then
    echo "[imu/shutdown] All processes stopped"
    exit 0
  fi
done

echo "[imu/shutdown] Forcing SIGKILL for remaining processes"
pkill -KILL -f "$PATTERN" || true
