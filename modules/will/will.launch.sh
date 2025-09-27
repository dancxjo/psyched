#!/bin/bash
set -euo pipefail
REPO_DIR="${REPO_DIR:-$(cd "$(dirname "$0")/../.." && pwd)}"
export REPO_DIR
HOST_SHORT="${HOST:-$(hostname -s)}"

HOST_YAML="${REPO_DIR}/hosts/${HOST_SHORT}/config/will.yaml"

if [ -f "$HOST_YAML" ]; then
	exec ros2 launch psyched_bt brain.launch.py --ros-args --params-file "$HOST_YAML"
else
	exec ros2 launch psyched_bt brain.launch.py
fi
