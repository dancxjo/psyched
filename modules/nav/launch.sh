#!/usr/bin/env bash
# Wrapper to launch the nav bringup using ros2 launch
set -euo pipefail

if [ -z "${ROS_DISTRO:-}" ]; then
  echo "Please source your ROS2 workspace (e.g. source /opt/ros/kilted/setup.bash and workspace install/setup.bash)"
  exit 1
fi

echo "Launching nav bringup..."
ros2 launch nav nav_bringup.launch.py
