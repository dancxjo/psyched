#!/usr/bin/env bash
set -euo pipefail

# Install ROS 2 and system dependencies required by the Kinect driver.
if ! command -v apt-get >/dev/null 2>&1; then
  echo "[eye/deps] apt-get not available; skipping dependency installation"
  exit 0
fi

ROS_DISTRO=${ROS_DISTRO:-kilted}
echo "[eye/deps] Installing Kinect dependencies for ROS distro '${ROS_DISTRO}'"

export DEBIAN_FRONTEND=noninteractive
SUDO_CMD=""
if command -v sudo >/dev/null 2>&1 && [[ ${EUID:-0} -ne 0 ]]; then
  SUDO_CMD="sudo"
fi

run_apt() {
  if [[ -n ${SUDO_CMD} ]]; then
    ${SUDO_CMD} "$@"
  else
    "$@"
  fi
}

run_apt apt-get update || true
run_apt apt-get install -y \
  ros-${ROS_DISTRO}-camera-calibration-parsers \
  ros-${ROS_DISTRO}-image-transport \
  ros-${ROS_DISTRO}-image-transport-plugins \
  ros-${ROS_DISTRO}-image-pipeline \
  ros-${ROS_DISTRO}-perception \
  ros-${ROS_DISTRO}-perception-pcl \
  ros-${ROS_DISTRO}-vision-msgs \
  ros-${ROS_DISTRO}-usb-cam \
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
  || true

if ! run_apt apt-get install -y ros-${ROS_DISTRO}-cv-bridge; then
  echo "[eye/deps] Warning: ros-${ROS_DISTRO}-cv-bridge failed to install." >&2
  echo "[eye/deps] You may need to resolve held packages or build cv_bridge from source." >&2
fi
