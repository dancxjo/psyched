#!/usr/bin/env bash
set -euo pipefail

PATTERN="ros2 launch psyched_nav nav_bringup.launch.py"
TIMEOUT=${TIMEOUT:-20}

if ! pgrep -f "${PATTERN}" >/dev/null 2>&1; then
  echo "[nav/shutdown] No launch process found"
  exit 0
fi

echo "[nav/shutdown] Sending SIGTERM to ${PATTERN}"
pkill -TERM -f "${PATTERN}" || true

for ((i = 0; i < TIMEOUT; i++)); do
  sleep 1
  if ! pgrep -f "${PATTERN}" >/dev/null 2>&1; then
    echo "[nav/shutdown] Launch process stopped"
    exit 0
  fi
done

echo "[nav/shutdown] Escalating to SIGKILL"
pkill -KILL -f "${PATTERN}" || true
