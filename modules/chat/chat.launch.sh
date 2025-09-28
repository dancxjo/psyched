#!/bin/bash
set -euo pipefail
REPO_DIR="${REPO_DIR:-$(cd "$(dirname "$0")/../.." && pwd)}"
export REPO_DIR
HOST_SHORT="${HOST:-$(hostname -s)}"

HOST_TOML="${REPO_DIR}/hosts/${HOST_SHORT}/config/chat.toml"
EXTRA_ARGS=()
if [ -f "$HOST_TOML" ]; then
  mapfile -t EXTRA_ARGS < <(python3 "$REPO_DIR/tools/launch_args.py" "$HOST_TOML")
fi

exec ros2 launch chat chat.launch.py "${EXTRA_ARGS[@]}"
