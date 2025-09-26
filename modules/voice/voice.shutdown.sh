#!/bin/bash
set -euo pipefail
ros2 topic pub /voice/say std_msgs/msg/String "{data: 'Goodbye everybody! I've got to go.'}" --once || true
PATTERN="ros2 launch voice voice.launch.py"
TIMEOUT=${TIMEOUT:-10}
mapfile -t pids < <(pgrep -f "$PATTERN" || true)
if [ ${#pids[@]} -eq 0 ]; then
  echo "[voice/shutdown] No matching processes found for pattern: $PATTERN"
  exit 0
fi

echo "[voice/shutdown] Stopping ${#pids[@]} process(es) for: $PATTERN (SIGTERM)"
for pid in "${pids[@]}"; do
  if [ "$pid" -ne $$ ]; then
    kill -TERM "$pid" 2>/dev/null || true
  fi
done

for ((i=0; i<TIMEOUT; i++)); do
  sleep 1
  mapfile -t still_alive < <(pgrep -f "$PATTERN" || true)
  if [ ${#still_alive[@]} -eq 0 ]; then
    echo "[voice/shutdown] All processes stopped"
    exit 0
  fi
done

echo "[voice/shutdown] Forcing stop (SIGKILL) for remaining processes"
for pid in "${still_alive[@]}"; do
  if [ -n "$pid" ]; then
    kill -KILL "$pid" 2>/dev/null || true
  fi
done
