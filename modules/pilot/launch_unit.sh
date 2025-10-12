#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="${REPO_DIR:-$(cd "${SCRIPT_DIR}/.." && pwd)}"
HOST_SHORT="${HOST:-$(hostname -s)}"

CONFIG_CANDIDATE="${PILOT_HOST_CONFIG:-}"
if [[ -z "${CONFIG_CANDIDATE}" ]]; then
  for suffix in json jsonc yaml yml toml; do
    candidate="${REPO_DIR}/hosts/${HOST_SHORT}.${suffix}"
    if [[ -f "${candidate}" ]]; then
      CONFIG_CANDIDATE="${candidate}"
      break
    fi
  done
fi

FRONTEND_ROOT="${PILOT_FRONTEND_ROOT:-${REPO_DIR}/modules/pilot/frontend}"

ARGS=("--listen-host" "${PILOT_LISTEN_HOST:-0.0.0.0}" "--listen-port" "${PILOT_LISTEN_PORT:-8088}")

if [[ -n "${CONFIG_CANDIDATE}" ]]; then
  ARGS+=("--host-config" "${CONFIG_CANDIDATE}")
fi

if [[ -d "${FRONTEND_ROOT}" ]]; then
  ARGS+=("--frontend-root" "${FRONTEND_ROOT}")
fi

if [[ -n "${PILOT_LOG_LEVEL:-}" ]]; then
  ARGS+=("--log-level" "${PILOT_LOG_LEVEL}")
fi

exec ros2 run pilot cockpit "${ARGS[@]}"
