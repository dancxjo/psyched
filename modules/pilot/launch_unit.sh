#!/usr/bin/env bash
set -euo pipefail

# Environment is set up by psh before calling this script
# PSYCHED_WORKSPACE_DIR and ROS environment should already be available

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
FRONTEND_DIR="${ROOT_DIR}/modules/pilot/frontend"
WORKSPACE_DIR="${PSYCHED_WORKSPACE_DIR:-${ROOT_DIR}/work}"

cleanup() {
  if [[ -n "${COCKPIT_PID:-}" ]] && kill -0 "${COCKPIT_PID}" >/dev/null 2>&1; then
    echo "Stopping Pilot cockpit backend (PID ${COCKPIT_PID})..."
    kill "${COCKPIT_PID}" 2>/dev/null || true
    wait "${COCKPIT_PID}" 2>/dev/null || true
  fi

  if [[ -n "${DENO_PID:-}" ]] && kill -0 "${DENO_PID}" >/dev/null 2>&1; then
    echo "Stopping Pilot frontend (PID ${DENO_PID})..."
    kill "${DENO_PID}" 2>/dev/null || true
    wait "${DENO_PID}" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

if ! command -v deno >/dev/null 2>&1; then
  echo "deno is not installed. Install deno before launching the pilot module." >&2
  exit 1
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 CLI not found. Source your ROS workspace before launching the pilot module." >&2
  exit 1
fi

echo "Starting Pilot cockpit backend..."
(
  cd "${ROOT_DIR}" &&
    ros2 run pilot cockpit --log-level "${PILOT_LOG_LEVEL:-info}"
) &
COCKPIT_PID=$!

sleep 2
if ! kill -0 "${COCKPIT_PID}" >/dev/null 2>&1; then
  echo "Pilot cockpit backend failed to start." >&2
  wait "${COCKPIT_PID}" || true
  exit 1
fi

echo "Going to frontend directory: ${FRONTEND_DIR}"
cd "${FRONTEND_DIR}"

echo "Starting Pilot frontend via deno task dev..."
deno task dev &
DENO_PID=$!

sleep 2
if ! kill -0 "${DENO_PID}" >/dev/null 2>&1; then
  echo "Pilot frontend failed to start." >&2
  wait "${DENO_PID}" || true
  exit 1
fi

wait -n
