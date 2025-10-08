#!/usr/bin/env bash
set -euo pipefail

PATTERN="ros2 launch psyched_bt brain.launch.py"
TIMEOUT=${TIMEOUT:-10}

if ! pgrep -f "${PATTERN}" >/dev/null 2>&1; then
  echo "[will/shutdown] No launch process running"
  exit 0
fi

echo "[will/shutdown] Sending SIGTERM to ${PATTERN}"
pkill -TERM -f "${PATTERN}" || true

for ((i = 0; i < TIMEOUT; i++)); do
  sleep 1
  if ! pgrep -f "${PATTERN}" >/dev/null 2>&1; then
    echo "[will/shutdown] Launch process stopped"
    exit 0
  fi
done

echo "[will/shutdown] Forcing SIGKILL"
pkill -KILL -f "${PATTERN}" || true
