#!/usr/bin/env bash
set -euo pipefail

PATTERNS=(
  "ros2 launch kinect_ros2 kinect.launch.py"
  "kinect_ros2_node"
)
TIMEOUT=${TIMEOUT:-10}

kill_with_pattern() {
  local pattern="$1"
  local timeout="$2"

  mapfile -t pids < <(pgrep -f "$pattern" || true)
  if [ ${#pids[@]} -eq 0 ]; then
    echo "[eye/shutdown] No matching processes found for pattern: $pattern"
    return 0
  fi

  echo "[eye/shutdown] Stopping ${#pids[@]} process(es) for: $pattern (SIGTERM)"
  for pid in "${pids[@]}"; do
    if [ "$pid" -ne $$ ]; then
      kill -TERM "$pid" 2>/dev/null || true
    fi
  done

  for ((i=0; i<timeout; i++)); do
    sleep 1
    mapfile -t still_alive < <(pgrep -f "$pattern" || true)
    if [ ${#still_alive[@]} -eq 0 ]; then
      echo "[eye/shutdown] All processes stopped for pattern: $pattern"
      return 0
    fi
  done

  echo "[eye/shutdown] Forcing stop (SIGKILL) for remaining processes"
  for pid in "${still_alive[@]}"; do
    if [ -n "$pid" ]; then
      kill -KILL "$pid" 2>/dev/null || true
    fi
  done
}

for p in "${PATTERNS[@]}"; do
  kill_with_pattern "$p" "$TIMEOUT"
done
