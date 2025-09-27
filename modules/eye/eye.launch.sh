#!/bin/bash
set -euo pipefail
REPO_DIR="${REPO_DIR:-$(cd "$(dirname "$0")/../.." && pwd)}"
export REPO_DIR
HOST_SHORT="${HOST:-$(hostname -s)}"

HOST_YAML="${REPO_DIR}/hosts/${HOST_SHORT}/config/eye.yaml"

if [ -f "$HOST_YAML" ]; then
	exec ros2 run kinect_ros2 kinect_ros2_node --ros-args --params-file "$HOST_YAML"
else
	exec ros2 run kinect_ros2 kinect_ros2_node
fi
