#!/bin/bash
set -euo pipefail
if command -v apt-get >/dev/null 2>&1; then
  ROS_DISTRO=${ROS_DISTRO:-kilted}
  sudo apt-get update || true
  sudo apt-get install -y \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-camera-calibration-parsers \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-image-transport-plugins \
    ros-${ROS_DISTRO}-image-pipeline \
    ros-${ROS_DISTRO}-perception \
    ros-${ROS_DISTRO}-perception-pcl \
    ros-${ROS_DISTRO}-vision-msgs \
    ros-${ROS_DISTRO}-usb-cam \
    libopencv-dev \
    python3-opencv \
    libglu1-mesa-dev \
    freeglut3-dev \
    mesa-common-dev \
    libogre-1.12-dev \
    ros-${ROS_DISTRO}-rviz2 \
    libgl1 \
    libegl1 \
    libxrandr2 \
    libxrandr-dev \
    libxinerama1 \
    libxinerama-dev \
    libxcursor1 \
    libxcursor-dev \
    libusb-1.0-0-dev \
    pkg-config || true
fi
