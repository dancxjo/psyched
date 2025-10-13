#!/usr/bin/env bash
set -euo pipefail

# Launch the Kinect ROS 2 driver with optional host-specific parameters.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="${REPO_DIR:-$(cd "${SCRIPT_DIR}/../.." && pwd)}"
HOST_SHORT="${HOST:-$(hostname -s)}"
PARAM_FILE="${REPO_DIR}/hosts/${HOST_SHORT}/config/eye.yaml"

CMD=("ros2" "run" "kinect_ros2" "kinect_ros2_node")
if [[ -f "${PARAM_FILE}" ]]; then
  CMD+=("--ros-args" "--params-file" "${PARAM_FILE}")
fi

exec "${CMD[@]}"
