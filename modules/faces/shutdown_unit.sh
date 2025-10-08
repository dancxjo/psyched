#!/usr/bin/env bash
set -euo pipefail

PATTERN="ros2 launch faces face_detector.launch.py"
TIMEOUT=${TIMEOUT:-10}

if ! pgrep -f "${PATTERN}" >/dev/null 2>&1; then
  echo "[faces/shutdown] No running launch process found"
  exit 0
fi

echo "[faces/shutdown] Sending SIGTERM to ${PATTERN}"
pkill -TERM -f "${PATTERN}" || true

for ((i = 0; i < TIMEOUT; i++)); do
  sleep 1
  if ! pgrep -f "${PATTERN}" >/dev/null 2>&1; then
    echo "[faces/shutdown] Launch process stopped"
    exit 0
  fi
done

echo "[faces/shutdown] Forcing SIGKILL"
pkill -KILL -f "${PATTERN}" || true
