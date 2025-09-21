#!/usr/bin/env bash
set -euo pipefail

# Config: source ../../config/<module>.env from real script location
REAL_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$REAL_PATH")"
MODULE_NAME="$(basename "$(dirname "$REAL_PATH")")"
CONF_FILE="$(cd "$SCRIPT_DIR/../.." && pwd)/config/${MODULE_NAME}.env"
if [ -f "$CONF_FILE" ]; then
  # shellcheck disable=SC1090
  . "$CONF_FILE"
fi

# If kinect_ros2 package provides a launch file, try launching it
LAUNCH_PKG="kinect_ros2"
LAUNCH_FILE="kinect.launch.py"

if ros2 pkg list | grep -qx "$LAUNCH_PKG"; then
  if ros2 launch "$LAUNCH_PKG" "$LAUNCH_FILE" 2>/dev/null; then
    exit 0
  fi
fi

echo "[eye/launch] Falling back to running kinect_ros2 node executable directly (if available)."
if command -v kinect_ros2_node >/dev/null 2>&1; then
  exec kinect_ros2_node
else
  echo "[eye/launch] Could not find launch file or node for kinect_ros2. Ensure workspace is built." >&2
  exit 1
fi
