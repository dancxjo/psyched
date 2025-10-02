#!/usr/bin/env bash
#
# install_ros2.sh - Provision ROS 2 on Ubuntu using the upstream apt packages.
#
# This script mirrors the `psh dep ros2` command. The ROS distribution can be
# overridden via the ROS_DISTRO environment variable.
#
# Usage examples:
#   ROS_DISTRO=kilted ./install_ros2.sh
#   ROS_DISTRO=rolling ./install_ros2.sh
#   ./install_ros2.sh            # uses ROS_DISTRO (defaults to kilted)
#
set -euo pipefail

ROS_DISTRO=${ROS_DISTRO:-kilted}
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

SUDO=(sudo)
if [[ $(id -u) -eq 0 ]]; then
  SUDO=()
elif ! command -v sudo >/dev/null 2>&1; then
  echo "This script requires root privileges or sudo to escalate." >&2
  exit 1
fi

echo "Provisioning ROS 2 ${ROS_DISTRO} using Debian packages..."

export LANG=en_US.UTF-8

"${SUDO[@]}" apt update
"${SUDO[@]}" apt install -y locales
"${SUDO[@]}" locale-gen en_US en_US.UTF-8
"${SUDO[@]}" update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

locale

"${SUDO[@]}" apt install -y software-properties-common
"${SUDO[@]}" add-apt-repository -y universe
"${SUDO[@]}" apt update
"${SUDO[@]}" apt install -y curl ca-certificates

echo "Setting up ROS 2 apt repository..."

ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')

curl -fsSL -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"

echo "Installing ROS 2 ${ROS_DISTRO} packages..."


# Install only minimal ROS2 base (no desktop/GUI/Qt dependencies)
"${SUDO[@]}" dpkg -i /tmp/ros2-apt-source.deb
"${SUDO[@]}" apt update
"${SUDO[@]}" apt upgrade -y

"${SUDO[@]}" apt install -y \
  ros-${ROS_DISTRO}-ros-base \
  ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
  python3-colcon-common-extensions \
  python3-rosdep

cargo install cargo-ament-build

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  "${SUDO[@]}" rosdep init
fi

# Note: desktop/GUI packages (rviz, Qt, etc.) are intentionally excluded for minimal install.
rosdep update

"${SUDO[@]}" tee /etc/profile.d/ros2-defaults.sh >/dev/null <<PROFILE
# ROS 2 defaults provisioned by install_ros2.sh
export LANG=en_US.UTF-8
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
PROFILE

"${SUDO[@]}" chmod 644 /etc/profile.d/ros2-defaults.sh

echo "ROS 2 ${ROS_DISTRO} installation completed with Cyclone DDS as the default RMW."
echo "Source /opt/ros/${ROS_DISTRO}/setup.bash to begin using ROS 2."

GENERATE_SCRIPT="${SCRIPT_DIR}/generate_ros_rust_bindings.sh"
if ! "${GENERATE_SCRIPT}"; then
  echo "Warning: failed to vendor ROS Rust message crates. Re-run ${GENERATE_SCRIPT} once Docker is available." >&2
fi
