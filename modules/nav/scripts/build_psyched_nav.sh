#!/usr/bin/env bash
set -euo pipefail
if [ "${BUILD:-true}" = "true" ]; then
  ROS_DISTRO=${ROS_DISTRO:-humble}
  if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    # shellcheck disable=SC1090
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
  fi
  if [ -f "install/setup.bash" ]; then
    # shellcheck disable=SC1090
    source "install/setup.bash"
  fi
  if command -v colcon >/dev/null 2>&1; then
    colcon build --packages-select psyched_nav --symlink-install || true
  fi
fi
