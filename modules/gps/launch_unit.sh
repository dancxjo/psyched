#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="${REPO_DIR:-$(cd "${SCRIPT_DIR}/../.." && pwd)}"
HOST_SHORT="${HOST:-$(hostname -s)}"

PARAM_FILE="${GPS_PARAMS_FILE:-}"
if [[ -z "${PARAM_FILE}" ]]; then
  LEGACY_YAML="${REPO_DIR}/hosts/${HOST_SHORT}/config/gps.yaml"
  if [[ -f "${LEGACY_YAML}" ]]; then
    PARAM_FILE="${LEGACY_YAML}"
  fi
fi

if [[ -n "${PARAM_FILE}" ]]; then
  exec ros2 launch ublox_gps ublox_gps.launch.py --ros-args --params-file "${PARAM_FILE}"
fi

ARGS=()
CONFIG_FILE="${GPS_CONFIG_FILE:-}" 
if [[ -z "${CONFIG_FILE}" ]]; then
  DEFAULT_HOST_TOML="${REPO_DIR}/hosts/${HOST_SHORT}.toml"
  if [[ -f "${DEFAULT_HOST_TOML}" ]]; then
    CONFIG_FILE="${DEFAULT_HOST_TOML}"
  fi
fi

if [[ -n "${CONFIG_FILE}" ]]; then
  mapfile -t HOST_ARGS < <(python3 "${REPO_DIR}/tools/launch_args.py" --module gps "${CONFIG_FILE}" || true)
  if [[ ${#HOST_ARGS[@]} -gt 0 ]]; then
    ARGS+=("${HOST_ARGS[@]}")
  fi
fi

if [[ -n "${GPS_FRAME_ID:-}" ]]; then
  ARGS+=("frame_id:=${GPS_FRAME_ID}")
fi
if [[ -n "${GPS_UART_PORT:-}" ]]; then
  ARGS+=("device:=${GPS_UART_PORT}")
fi
if [[ -n "${GPS_BAUD_RATE:-}" ]]; then
  ARGS+=("baudrate:=${GPS_BAUD_RATE}")
fi

exec ros2 launch ublox_gps ublox_gps.launch.py "${ARGS[@]}"
