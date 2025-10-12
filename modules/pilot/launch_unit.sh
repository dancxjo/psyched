#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="${REPO_DIR:-$(cd "${SCRIPT_DIR}/../.." && pwd)}"
export REPO_DIR
HOST_SHORT="${HOST:-$(hostname -s)}"
PID_DIR="${REPO_DIR}/work/run/pilot"
mkdir -p "${PID_DIR}"

start_background() {
  local key="$1"
  shift
  local log="${PID_DIR}/${key}.log"
  local pid_file="${PID_DIR}/${key}.pid"
  rm -f "${pid_file}"
  echo "[pilot/launch] Starting ${key}: $*" >&2
  "$@" >>"${log}" 2>&1 &
  local child=$!
  echo "${child}" > "${pid_file}"
}

if [[ -f "${REPO_DIR}/work/install/setup.bash" ]]; then
  # colcon's setup scripts read unset vars, so relax -u while sourcing.
  set +u
  # shellcheck disable=SC1091
  source "${REPO_DIR}/work/install/setup.bash"
  set -u
fi

CONFIG_CANDIDATE="${PILOT_HOST_CONFIG:-}"
if [[ -z "${CONFIG_CANDIDATE}" ]]; then
  candidate="${REPO_DIR}/hosts/${HOST_SHORT}.toml"
  if [[ -f "${candidate}" ]]; then
    CONFIG_CANDIDATE="${candidate}"
  fi
elif [[ ! -f "${CONFIG_CANDIDATE}" ]]; then
  CONFIG_CANDIDATE=""
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
ROSBRIDGE_HOST="${PILOT_ROSBRIDGE_BIND_HOST:-0.0.0.0}"
ROSBRIDGE_PORT="${PILOT_ROSBRIDGE_PORT:-9090}"
ROSBRIDGE_URI="${PILOT_ROSBRIDGE_URI:-ws://127.0.0.1:${ROSBRIDGE_PORT}}"
TF2_FREQUENCY="${PILOT_TF2_FREQUENCY:-30.0}"
VIDEO_HOST="${PILOT_VIDEO_BIND_HOST:-0.0.0.0}"
VIDEO_PORT="${PILOT_VIDEO_PORT:-8089}"

export PILOT_BRIDGE_MODE="${PILOT_BRIDGE_MODE:-rosbridge}"
export PILOT_ROSBRIDGE_URI="${ROSBRIDGE_URI}"
export PILOT_VIDEO_PORT="${VIDEO_PORT}"
export PILOT_VIDEO_BASE="${PILOT_VIDEO_BASE:-http://127.0.0.1:${VIDEO_PORT}}"

start_background "rosbridge" ros2 run rosbridge_server rosbridge_websocket \
  --ros-args \
  -p port:="${ROSBRIDGE_PORT}" \
  -p address:="${ROSBRIDGE_HOST}" \
  -p use_compression:="false"

start_background "tf2_web_republisher" ros2 run tf2_web_republisher tf2_web_republisher \
  --ros-args \
  -p queue_size:=10 \
  -p frequency:="${TF2_FREQUENCY}"

start_background "web_video_server" ros2 run web_video_server web_video_server \
  --ros-args \
  -p address:="${VIDEO_HOST}" \
  -p port:="${VIDEO_PORT}" \
  -p default_streams:="ros_compressed" \
  -p server_threads:=4

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
