#!/bin/bash
set -euo pipefail
exec ros2 launch create_bringup create_1.launch

REPO_DIR="${REPO_DIR:-$(cd "$(dirname "$0")/../.." && pwd)}"
export REPO_DIR
