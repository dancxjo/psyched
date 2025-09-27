#!/bin/bash
set -euo pipefail
ROS_DISTRO=${ROS_DISTRO:-kilted}
HEADER="/opt/ros/${ROS_DISTRO}/include/cv_bridge/cv_bridge.hpp"
REPO="src/vision_opencv"

# The ROS packaging channel now ships the cv_bridge headers we previously
# overlaid from source. Prefer the apt-provided artefacts and only fall back to
# the git checkout when the packages cannot be installed (for example on
# minimal images without ROS repositories configured).
ensure_apt_cv_bridge() {
  if ! command -v apt-get >/dev/null 2>&1; then
    return 1
  fi

  # Attempt to install the vision_opencv metapackage; this pulls in cv_bridge,
  # image_geometry, and the rest of the stack that historically required the
  # source overlay. Using --no-install-recommends keeps the footprint minimal
  # while still delivering the headers we care about.
  sudo apt-get update -y >/dev/null 2>&1 || true
  if sudo apt-get install -y --no-install-recommends \
      "ros-${ROS_DISTRO}-vision-opencv" >/dev/null 2>&1; then
    return 0
  fi

  # Fallback: try the individual packages that back the metapackage in case the
  # exact metapackage name differs on derivative distros.
  sudo apt-get install -y --no-install-recommends \
    "ros-${ROS_DISTRO}-cv-bridge" \
    "ros-${ROS_DISTRO}-image-geometry" >/dev/null 2>&1 || return 1
  return 0
}

if [ -f "$HEADER" ]; then
  # If an old overlay remains in the workspace, remove it so colcon stops
  # preferring the source copy over the apt-installed package.
  if [ -d "$REPO" ]; then
    rm -rf "$REPO"
  fi
  exit 0
fi

if ensure_apt_cv_bridge && [ -f "$HEADER" ]; then
  if [ -d "$REPO" ]; then
    rm -rf "$REPO"
  fi
  exit 0
fi

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
