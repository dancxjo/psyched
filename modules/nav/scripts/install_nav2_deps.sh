#!/usr/bin/env bash
set -euo pipefail
if command -v apt-get >/dev/null 2>&1; then
  ROS_DISTRO=${ROS_DISTRO:-humble}
  PKGS=(
    "ros-${ROS_DISTRO}-nav2-bringup"
    "ros-${ROS_DISTRO}-nav2-amcl"
    "ros-${ROS_DISTRO}-nav2-controller"
    "ros-${ROS_DISTRO}-nav2-core"
    "ros-${ROS_DISTRO}-nav2-behavior-tree"
    "ros-${ROS_DISTRO}-nav2-costmap-2d"
    "ros-${ROS_DISTRO}-nav2-map-server"
    "ros-${ROS_DISTRO}-nav2-navfn-planner"
    "ros-${ROS_DISTRO}-nav2-smac-planner"
    "ros-${ROS_DISTRO}-nav2-lifecycle-manager"
    "ros-${ROS_DISTRO}-pcl-ros"
    "libpcl-dev"
  )
  TO_INSTALL=()
  for pkg in "${PKGS[@]}"; do
    if apt-cache show "$pkg" >/dev/null 2>&1; then
      TO_INSTALL+=("$pkg")
    fi
  done
  if [ ${#TO_INSTALL[@]} -gt 0 ]; then
    sudo apt-get update || true
    sudo apt-get install -y "${TO_INSTALL[@]}" || true
  fi
fi
