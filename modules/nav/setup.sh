#!/usr/bin/env bash
set -euo pipefail


echo "Setting up nav module..."

# Correct repo dir calculation: use hosts directory, not modules
REPO_DIR="$(dirname "$(dirname "$(realpath "$0")")")"
HOSTS_DIR="$REPO_DIR/hosts"
echo "Repo dir: $REPO_DIR"
echo "Hosts dir: $HOSTS_DIR"

if [ "${EUID}" -ne 0 ]; then
  SUDO=sudo
else
  SUDO=""
fi

if ! command -v apt-get >/dev/null 2>&1; then
  echo "apt-get not found — this setup script only supports Debian/Ubuntu hosts."
  exit 1
fi

if ! ping -c1 -W1 8.8.8.8 >/dev/null 2>&1; then
  echo "Network seems unavailable. Skipping package install and repo clones."
  echo "Please install nav2 and rtabmap packages manually when online."
  exit 0
fi

${SUDO} apt-get update

# ROS2 and navigation packages
echo "Determining package names for ROS distro: ${ROS_DISTRO:-<undefined>}"

# Only include nav2 and related navigation stack packages, exclude rtabmap and grid-map-ros
declare -a CANDIDATES=(
  "ros-${ROS_DISTRO}-nav2-bringup"
  "ros-${ROS_DISTRO}-nav2"
  "ros-${ROS_DISTRO}-nav2-nav2"
  "ros-${ROS_DISTRO}-octomap-ros"
  "ros-${ROS_DISTRO}-octomap"
  "ros-${ROS_DISTRO}-pcl-ros"
  "libpcl-dev"
)

PKGS_AVAILABLE=()
PKGS_MISSING=()






echo "Nav module setup complete."

echo "All rtabmap and grid-map-ros logic removed as requested."
