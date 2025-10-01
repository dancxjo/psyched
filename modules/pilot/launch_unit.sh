#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
FRONTEND_DIR="${ROOT_DIR}/modules/pilot/frontend"

# shellcheck disable=SC1090
if [[ -f "${ROOT_DIR}/install/setup.bash" ]]; then
  source "${ROOT_DIR}/install/setup.bash"
elif [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

if ! command -v cargo >/dev/null 2>&1; then
  echo "cargo is not installed. Install Rust before launching the pilot module." >&2
  exit 1
fi

if ! command -v deno >/dev/null 2>&1; then
  echo "deno is not installed. Install deno before launching the pilot module." >&2
  exit 1
fi

echo "Starting Psyched cockpit backend..."
(
  cd "${ROOT_DIR}" &&
    cargo run --package psyched --bin cockpit
) &
COCKPIT_PID=$!

sleep 2
if ! kill -0 "${COCKPIT_PID}" >/dev/null 2>&1; then
  echo "Psyched cockpit backend failed to start." >&2
  wait "${COCKPIT_PID}" || true
  exit 1
fi

cleanup() {
  if kill -0 "${COCKPIT_PID}" >/dev/null 2>&1; then
    echo "Stopping Psyched cockpit backend (PID ${COCKPIT_PID})..."
    kill "${COCKPIT_PID}" 2>/dev/null || true
    wait "${COCKPIT_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

cd "${FRONTEND_DIR}"

echo "Starting Psyched pilot frontend via deno task start..."
exec deno task start
