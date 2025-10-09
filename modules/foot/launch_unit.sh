#!/usr/bin/env bash
set -euo pipefail

echo "[foot/launch] Starting create_bringup launcher..."

cleanup() {
  if [[ -n "${FOOT_PID:-}" ]] && kill -0 "${FOOT_PID}" >/dev/null 2>&1; then
    echo "[foot/launch] Stopping create_bringup launcher (PID ${FOOT_PID})"
    kill "${FOOT_PID}" 2>/dev/null || true
    wait "${FOOT_PID}" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

ros2 launch create_bringup create_1.launch &
FOOT_PID=$!

wait "${FOOT_PID}"
