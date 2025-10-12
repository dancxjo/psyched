#!/usr/bin/env bash
set -euo pipefail

PATTERN="ros2 launch felt felt.launch.py"
TIMEOUT=${TIMEOUT:-10}

if ! pgrep -f "${PATTERN}" >/dev/null 2>&1; then
  echo "[felt/shutdown] No matching processes found for pattern: ${PATTERN}"
  exit 0
fi

echo "[felt/shutdown] Sending SIGTERM to processes for: ${PATTERN}"
pkill -TERM -f "${PATTERN}" || true

for ((i = 0; i < TIMEOUT; i++)); do
  sleep 1
  if ! pgrep -f "${PATTERN}" >/dev/null 2>&1; then
    echo "[felt/shutdown] All processes stopped"
    exit 0
  fi
done

echo "[felt/shutdown] Forcing SIGKILL for remaining processes"
pkill -KILL -f "${PATTERN}" || true
