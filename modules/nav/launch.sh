#!/usr/bin/env bash
# Wrapper to launch the nav bringup using ros2 launch
set -euo pipefail

if [ -z "${ROS_DISTRO:-}" ]; then
  echo "Please source your ROS2 workspace (e.g. source /opt/ros/kilted/setup.bash and workspace install/setup.bash)"
  exit 1
fi

echo "Launching nav bringup (psyched_nav)..."
# Ensure our manually-installed package prefix is visible to ROS 2 tools
export AMENT_PREFIX_PATH="$(pwd)/install/psyched_nav:${AMENT_PREFIX_PATH:-}"
export ROS_PACKAGE_PATH="$(pwd)/install/psyched_nav/share:${ROS_PACKAGE_PATH:-}"
ros2 launch psyched_nav nav_bringup.launch.py
