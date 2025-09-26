#!/bin/bash
set -euo pipefail
ROS_DISTRO=${ROS_DISTRO:-kilted}
HEADER="/opt/ros/${ROS_DISTRO}/include/cv_bridge/cv_bridge.hpp"
REPO="src/vision_opencv"
if [ ! -f "$HEADER" ]; then
  if [ ! -d "$REPO/.git" ]; then
    git clone https://github.com/ros-perception/vision_opencv.git "$REPO"
  fi
  if [ -d "$REPO/.git" ]; then
    git -C "$REPO" fetch --tags --force >/dev/null 2>&1 || true
    if git -C "$REPO" rev-parse --verify "$ROS_DISTRO" >/dev/null 2>&1; then
      git -C "$REPO" checkout "$ROS_DISTRO" >/dev/null 2>&1 || true
    elif git -C "$REPO" rev-parse --verify ros2 >/dev/null 2>&1; then
      git -C "$REPO" checkout ros2 >/dev/null 2>&1 || true
    fi
    rm -f "$REPO/COLCON_IGNORE" || true
  fi
fi
