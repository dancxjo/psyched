#!/usr/bin/env bash
set -euo pipefail

# Gracefully stop Kinect ROS 2 processes spawned by the launch script.
TIMEOUT=${TIMEOUT:-10}
PATTERNS=(
  "ros2 run kinect_ros2 kinect_ros2_node"
  "kinect_ros2_node"
)

stop_matching() {
  local pattern=$1
  mapfile -t pids < <(pgrep -f "${pattern}" || true)
  if [[ ${#pids[@]} -eq 0 ]]; then
    echo "[eye/shutdown] No matching processes found for pattern: ${pattern}"
    return
  fi

  echo "[eye/shutdown] Stopping ${#pids[@]} process(es) for: ${pattern} (SIGTERM)"
  for pid in "${pids[@]}"; do
    if [[ ${pid} -ne $$ ]]; then
      kill -TERM "${pid}" 2>/dev/null || true
    fi
  done

  for ((i = 0; i < TIMEOUT; i++)); do
    sleep 1
    mapfile -t remaining < <(pgrep -f "${pattern}" || true)
    if [[ ${#remaining[@]} -eq 0 ]]; then
      echo "[eye/shutdown] All processes stopped for pattern: ${pattern}"
      return
    fi
  done

  echo "[eye/shutdown] Forcing stop (SIGKILL) for remaining processes"
  for pid in "${remaining[@]}"; do
    if [[ -n ${pid} ]]; then
      kill -KILL "${pid}" 2>/dev/null || true
    fi
  done
}

for pattern in "${PATTERNS[@]}"; do
  stop_matching "${pattern}"
done
