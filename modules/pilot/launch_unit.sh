#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
FRONTEND_DIR="${ROOT_DIR}/modules/pilot/frontend"
WORKSPACE_ENV="${ROOT_DIR}/workspace_env.sh"

if [[ -f "${WORKSPACE_ENV}" ]]; then
  # shellcheck disable=SC1090
  source "${WORKSPACE_ENV}"
fi

WORKSPACE_DIR="${PSYCHED_WORKSPACE_DIR:-${ROOT_DIR}/work}"
WORKSPACE_SRC="${PSYCHED_WORKSPACE_SRC:-${WORKSPACE_DIR}/src}"
COCKPIT_MANIFEST="${WORKSPACE_SRC}/pilot/Cargo.toml"

# shellcheck disable=SC1090
# Temporary disable nounset before sourcing scripts that expect unset vars.
source_with_nounset_guard() {
  local script_path=$1
  set +u
  source "${script_path}"
  set -u
}

if [[ -f "${ROOT_DIR}/install/setup.bash" ]]; then
  source_with_nounset_guard "${ROOT_DIR}/install/setup.bash"
elif [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  source_with_nounset_guard "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

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

if ! command -v cargo >/dev/null 2>&1; then
  echo "cargo is not installed. Install Rust before launching the pilot module." >&2
  exit 1
fi

if ! command -v deno >/dev/null 2>&1; then
  echo "deno is not installed. Install deno before launching the pilot module." >&2
  exit 1
fi

if [[ ! -f "${COCKPIT_MANIFEST}" ]]; then
  cat >&2 <<EOF
Pilot cockpit manifest not found at ${COCKPIT_MANIFEST}.
Ensure you've prepared the workspace (e.g. run 'psh clean' or 'psh mod setup pilot').
EOF
  exit 1
fi

echo "Starting Pilot cockpit backend..."
(
  cd "${ROOT_DIR}" &&
    cargo run --manifest-path "${COCKPIT_MANIFEST}" --bin cockpit
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
