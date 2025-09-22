#!/usr/bin/env bash
set -euo pipefail

echo "Setting up nav module..."

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

# Expanded candidate package name templates to try (some distros / repos name things differently)
declare -a CANDIDATES=(
  "ros-${ROS_DISTRO}-nav2-bringup"
  "ros-${ROS_DISTRO}-nav2"
  "ros-${ROS_DISTRO}-nav2-nav2"
  "ros-${ROS_DISTRO}-rtabmap-ros"
  "ros-${ROS_DISTRO}-rtabmap"
  "ros-${ROS_DISTRO}-rtabmap-ros-plugins"
  "ros-${ROS_DISTRO}-rtabmap"
  "ros-${ROS_DISTRO}-octomap-ros"
  "ros-${ROS_DISTRO}-octomap"
  "ros-${ROS_DISTRO}-pcl-ros"
  "libpcl-dev"
  "rtabmap"
  "rtabmap-tools"
)

PKGS_AVAILABLE=()
PKGS_MISSING=()

for pkg in "${CANDIDATES[@]}"; do
  # Only add each package once (keep order)
  if printf '%s\n' "${PKGS_AVAILABLE[@]}" | grep -qx "${pkg}"; then
    continue
  fi

  if apt-cache show "${pkg}" >/dev/null 2>&1; then
    PKGS_AVAILABLE+=("${pkg}")
  else
    PKGS_MISSING+=("${pkg}")
  fi
done

if [ ${#PKGS_AVAILABLE[@]} -gt 0 ]; then
  echo "Installing available packages: ${PKGS_AVAILABLE[*]}"
  ${SUDO} apt-get install -y "${PKGS_AVAILABLE[@]}"
else
  echo "No prebuilt ROS packages from candidate list were found in apt for distro '${ROS_DISTRO}'."
  echo "Missing candidates: ${PKGS_MISSING[*]}"
  echo "You'll need to install Nav2/RTAB-Map from source or enable the appropriate Debian/ROS apt repository."
  echo "By default this script will NOT clone the rtabmap_ros GitHub repo. To allow cloning, re-run with --allow-clone"
fi

# If rtabmap_ros isn't available as a package, offer to clone only when explicitly allowed
RTAB_PKG="ros-${ROS_DISTRO}-rtabmap-ros"
if ! dpkg -s "${RTAB_PKG}" >/dev/null 2>&1 && ! apt-cache show "${RTAB_PKG}" >/dev/null 2>&1; then
  # Determine repository root like other module setup scripts
  if REPO_DIR_GIT_ROOT=$(git -C "$(dirname "${BASH_SOURCE[0]}")" rev-parse --show-toplevel 2>/dev/null); then
    REPO_DIR="$REPO_DIR_GIT_ROOT"
  else
    REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
  fi
  SOURCE_DIR="${REPO_DIR}/src"
  echo "Binary package for ${RTAB_PKG} not found, cloning source into ${SOURCE_DIR}/rtabmap_ros"
  mkdir -p "${SOURCE_DIR}"
  if [ ! -d "${SOURCE_DIR}/rtabmap_ros" ]; then
    if command -v git >/dev/null 2>&1; then
      git clone https://github.com/introlab/rtabmap_ros.git "${SOURCE_DIR}/rtabmap_ros" || echo "git clone failed (network?)"
    else
      echo "git not available; cannot clone rtabmap_ros source."
    fi
  else
    echo "rtabmap_ros already present in ${SOURCE_DIR}/rtabmap_ros"
  fi
fi

mkdir -p "${SOURCE_DIR:-./src}"

# We only clone the upstream rtabmap_ros repository once (into '${SOURCE_DIR}/rtabmap_ros').
# Do not clone a second copy as 'rtabmap_ros_examples' which creates duplicate package names.
if [ ! -d "${SOURCE_DIR:-./src}/rtabmap_ros" ]; then
  if command -v git >/dev/null 2>&1; then
    git clone https://github.com/introlab/rtabmap_ros.git "${SOURCE_DIR}/rtabmap_ros" || echo "git clone failed (network?)"
  else
    echo "git not available; cannot clone rtabmap_ros source."
  fi
else
  echo "rtabmap_ros already present in ${SOURCE_DIR:-./src}/rtabmap_ros"
fi

echo "Nav module setup complete."
