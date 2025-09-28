#!/bin/bash
set -euo pipefail
REPO_DIR="${REPO_DIR:-$(cd "$(dirname "$0")/../.." && pwd)}"
export REPO_DIR
HOST_SHORT="${HOST:-$(hostname -s)}"

HOST_YAML="${REPO_DIR}/hosts/${HOST_SHORT}/config/ear.yaml"

if [ -f "$HOST_YAML" ]; then
  export PSH_MODULE_CONFIG="$HOST_YAML"
fi

exec ros2 launch ear ear.launch.py
