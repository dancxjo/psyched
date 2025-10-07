#!/bin/bash
set -euo pipefail
PATTERNS=(
  "ros2 launch mpu6050driver mpu6050driver_launch.py"
  "ros2 run imu_filter_madgwick imu_filter_madgwick_node"
)

for PATTERN in "${PATTERNS[@]}"; do
  if pgrep -f "$PATTERN" >/dev/null 2>&1; then
    echo "[imu/shutdown] Stopping $PATTERN"
    pkill -TERM -f "$PATTERN" || true
  fi
done

# Wait up to 10s for shutdown
for i in {1..10}; do
  sleep 1
  all_done=true
  for PATTERN in "${PATTERNS[@]}"; do
    if pgrep -f "$PATTERN" >/dev/null 2>&1; then
      all_done=false
    fi
  done
  $all_done && echo "[imu/shutdown] All processes stopped" && exit 0
done

# Force kill if still running
for PATTERN in "${PATTERNS[@]}"; do
  pkill -KILL -f "$PATTERN" || true
done
echo "[imu/shutdown] Forced kill"
