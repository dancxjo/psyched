#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="${REPO_DIR:-$(cd "${SCRIPT_DIR}/../.." && pwd)}"
export REPO_DIR
HOST_SHORT="${HOST:-$(hostname -s)}"

if [[ -f "${REPO_DIR}/work/install/setup.bash" ]]; then
  # colcon's setup scripts read unset vars, so relax -u while sourcing.
  set +u
  # shellcheck disable=SC1091
  source "${REPO_DIR}/work/install/setup.bash"
  set -u
fi

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

FRONTEND_ROOT="${PILOT_FRONTEND_ROOT:-${REPO_DIR}/modules/pilot/packages/pilot/pilot/frontend}"
export PILOT_FRONTEND_ROOT="${FRONTEND_ROOT}"
if [[ ! -d "${FRONTEND_ROOT}" ]]; then
  echo "ERROR: Could not find cockpit frontend assets." >&2
  exit 1
fi

if [[ -z "${PILOT_MODULES_ROOT:-}" ]]; then
  export PILOT_MODULES_ROOT="${REPO_DIR}/modules"
fi

ARGS=("--listen-host" "${PILOT_LISTEN_HOST:-0.0.0.0}" "--listen-port" "${PILOT_LISTEN_PORT:-8088}")

if [[ -n "${CONFIG_CANDIDATE}" ]]; then
  ARGS+=("--host-config" "${CONFIG_CANDIDATE}")
fi

ARGS+=("--frontend-root" "${FRONTEND_ROOT}")

if [[ -n "${PILOT_LOG_LEVEL:-}" ]]; then
  ARGS+=("--log-level" "${PILOT_LOG_LEVEL}")
fi

if [[ -f "${REPO_DIR}/work/install/pilot/share/pilot/local_setup.bash" ]]; then
  set +u
  # shellcheck disable=SC1091
  source "${REPO_DIR}/work/install/pilot/share/pilot/local_setup.bash"
  set -u
fi

exec ros2 run pilot pilot_cockpit "${ARGS[@]}"
