#!/bin/bash
set -euo pipefail
exec ros2 run kinect_ros2 kinect_ros2_node

REPO_DIR="${REPO_DIR:-$(cd "$(dirname "$0")/../.." && pwd)}"
export REPO_DIR
