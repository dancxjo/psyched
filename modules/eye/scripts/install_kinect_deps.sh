#!/bin/bash
set -euo pipefail
if command -v apt-get >/dev/null 2>&1; then
  ROS_DISTRO=${ROS_DISTRO:-kilted}
  sudo apt-get update || true
  # Install system OpenCV and supporting libraries first. We avoid putting
  # ros-${ROS_DISTRO}-cv-bridge in this main list because some systems show
  # held/broken package issues for the ROS cv-bridge binary package (eg.
  # libboost-python variant mismatches). Installing libopencv-dev +
  # python3-opencv is sufficient for building kinect_ros2 from source in
  # most cases.
  sudo apt-get install -y \
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

  # Try to install the ROS binary for cv_bridge separately. If it fails due
  # to held/broken packages (common when libboost-python variants mismatch),
  # we don't treat that as fatal here — building cv_bridge from source is an
  # alternate path (see README or run 'psh mod setup --full' for extra steps).
  if ! sudo apt-get install -y ros-${ROS_DISTRO}-cv-bridge; then
    echo "Warning: ros-${ROS_DISTRO}-cv-bridge failed to install."
    echo "If you need binary cv_bridge, resolve held packages (eg. libboost-python)"
    echo "or consider building cv_bridge from source in your workspace."
  fi
fi
