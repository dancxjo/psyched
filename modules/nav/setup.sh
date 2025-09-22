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

# Candidate package name templates to try (some distros / repos name things differently)
declare -a CANDIDATES=(
  "ros-${ROS_DISTRO}-nav2-bringup"
  "ros-${ROS_DISTRO}-nav2"
  "ros-${ROS_DISTRO}-nav2-nav2"
  "ros-${ROS_DISTRO}-rtabmap-ros"
  "ros-${ROS_DISTRO}-rtabmap"
  "ros-${ROS_DISTRO}-octomap-ros"
  "ros-${ROS_DISTRO}-octomap"
  "ros-${ROS_DISTRO}-pcl-ros"
  "libpcl-dev"
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
fi

# If rtabmap_ros isn't available as a package, clone the source so it can be built in the workspace
if ! dpkg -s "ros-${ROS_DISTRO}-rtabmap-ros" >/dev/null 2>&1 && ! apt-cache show "ros-${ROS_DISTRO}-rtabmap-ros" >/dev/null 2>&1; then
  echo "Binary package for rtabmap_ros not found, cloning source into modules/nav/external/rtabmap_ros"
  mkdir -p modules/nav/external
  pushd modules/nav/external >/dev/null
  if [ ! -d rtabmap_ros ]; then
    if command -v git >/dev/null 2>&1; then
      git clone https://github.com/introlab/rtabmap_ros.git rtabmap_ros || echo "git clone failed (network?)"
    else
      echo "git not available; cannot clone rtabmap_ros source."
    fi
  else
    echo "rtabmap_ros already present in modules/nav/external/rtabmap_ros"
  fi
  popd >/dev/null
fi

echo "Cloning helpful repos into modules/nav/external (if network allowed)"
mkdir -p modules/nav/external
pushd modules/nav/external >/dev/null

if [ ! -d rtabmap_ros_examples ]; then
  git clone https://github.com/introlab/rtabmap_ros.git rtabmap_ros_examples || true
fi

popd >/dev/null

echo "Nav module setup complete."
