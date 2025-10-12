#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="${REPO_DIR:-$(cd "${SCRIPT_DIR}/../.." && pwd)}"
HOST_SHORT="${HOST:-$(hostname -s)}"

ARGS=()
CONFIG_FILE="${NAV_CONFIG_FILE:-${REPO_DIR}/hosts/${HOST_SHORT}.toml}"

if [[ -n "${CONFIG_FILE}" && -f "${CONFIG_FILE}" ]]; then
  mapfile -t HOST_ARGS < <(python3 "${REPO_DIR}/tools/launch_args.py" --module nav "${CONFIG_FILE}" || true)
  if [[ ${#HOST_ARGS[@]} -gt 0 ]]; then
    ARGS+=("${HOST_ARGS[@]}")
  fi
fi

if [[ -n "${NAV_KINECT_RGB_TOPIC:-}" ]]; then
  ARGS+=("kinect_rgb_topic:=${NAV_KINECT_RGB_TOPIC}")
fi
if [[ -n "${NAV_KINECT_DEPTH_TOPIC:-}" ]]; then
  ARGS+=("kinect_depth_topic:=${NAV_KINECT_DEPTH_TOPIC}")
fi
if [[ -n "${NAV_CAMERA_FRAME:-}" ]]; then
  ARGS+=("camera_frame:=${NAV_CAMERA_FRAME}")
fi

exec ros2 launch psyched_nav nav_bringup.launch.py "${ARGS[@]}"
