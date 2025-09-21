#!/usr/bin/env bash
set -euo pipefail

# Gracefully stop the imu module's ros2 launch
PATTERN="ros2 launch mpu6050driver mpu6050driver_launch.py"
TIMEOUT=${TIMEOUT:-10}

kill_with_pattern() {
  local pattern="$1"
  local timeout="$2"

  mapfile -t pids < <(pgrep -f "$pattern" || true)
  if [ ${#pids[@]} -eq 0 ]; then
    echo "[imu/shutdown] No matching processes found for pattern: $pattern"
    return 0
  fi

  echo "[imu/shutdown] Stopping ${#pids[@]} process(es) for: $pattern (SIGTERM)"
  for pid in "${pids[@]}"; do
    if [ "$pid" -ne $$ ]; then
      kill -TERM "$pid" 2>/dev/null || true
    fi
  done

  for ((i=0; i<timeout; i++)); do
    sleep 1
    mapfile -t still_alive < <(pgrep -f "$pattern" || true)
    if [ ${#still_alive[@]} -eq 0 ]; then
      echo "[imu/shutdown] All processes stopped"
      return 0
    fi
  done

  echo "[imu/shutdown] Forcing stop (SIGKILL) for remaining processes"
  for pid in "${still_alive[@]}"; do
    if [ -n "$pid" ]; then
      kill -KILL "$pid" 2>/dev/null || true
    fi
  done
}

kill_with_pattern "$PATTERN" "$TIMEOUT"
