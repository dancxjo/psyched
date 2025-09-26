#!/bin/bash
set -euo pipefail
if [ -z "${ROS_DISTRO:-}" ]; then
  echo "Please source your ROS2 workspace (e.g. source /opt/ros/kilted/setup.bash and workspace install/setup.bash)"
  exit 1
fi

echo "Launching nav bringup (psyched_nav)..."
REPO_DIR="${REPO_DIR:-$(cd "$(dirname "$0")/../.." && pwd)}"
export REPO_DIR
export AMENT_PREFIX_PATH="$REPO_DIR/install/psyched_nav:${AMENT_PREFIX_PATH:-}"
export ROS_PACKAGE_PATH="$(pwd)/install/psyched_nav/share:${ROS_PACKAGE_PATH:-}"
exec ros2 launch psyched_nav nav_bringup.launch.py
