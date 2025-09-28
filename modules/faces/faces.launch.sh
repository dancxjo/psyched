#!/bin/bash
set -euo pipefail
REPO_DIR="${REPO_DIR:-$(cd "$(dirname "$0")/../.." && pwd)}"
export REPO_DIR
HOST_SHORT="${HOST:-$(hostname -s)}"

HOST_YAML="${REPO_DIR}/hosts/${HOST_SHORT}/config/faces.yaml"

if [ -f "$HOST_YAML" ]; then
  exec ros2 launch faces face_detector.launch.py --ros-args --params-file "$HOST_YAML"
else
  exec ros2 launch faces face_detector.launch.py
fi
