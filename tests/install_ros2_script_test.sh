#!/usr/bin/env bash
# Behavioural check ensuring the ROS 2 installer guards against known dpkg conflicts.
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROOT_DIR=$(cd "${SCRIPT_DIR}/.." && pwd)
ROS_INSTALLER="${ROOT_DIR}/tools/provision/install_ros2.sh"

if [[ ! -f "${ROS_INSTALLER}" ]]; then
  echo "Expected installer script at ${ROS_INSTALLER} is missing." >&2
  exit 1
fi

if ! grep -Fq 'apt-get remove -y python3-catkin-pkg' "${ROS_INSTALLER}"; then
  echo "install_ros2.sh must remove python3-catkin-pkg before installing ROS 2 base packages." >&2
  exit 1
fi

REMOVE_LINE=$(grep -Fn 'apt-get remove -y python3-catkin-pkg' "${ROS_INSTALLER}" | head -n1 | cut -d: -f1)

set +o pipefail
INSTALL_LINE=$(grep -Fn 'ros-${ROS_DISTRO:-\${ROS_DISTRO}}-ros-base' "${ROS_INSTALLER}" | head -n1 | cut -d: -f1)
set -o pipefail

if [[ -z "${INSTALL_LINE}" ]]; then
  INSTALL_LINE=$(grep -Fn 'ros-${ROS_DISTRO}-ros-base' "${ROS_INSTALLER}" | head -n1 | cut -d: -f1)
fi

if [[ -z "${INSTALL_LINE}" ]]; then
  echo "Failed to detect ros-base installation line in install_ros2.sh." >&2
  exit 1
fi

if (( REMOVE_LINE >= INSTALL_LINE )); then
  echo "python3-catkin-pkg removal must happen before ros-base installation to avoid dpkg conflicts." >&2
  exit 1
fi

if ! grep -Fq 'ros-${ROS_DISTRO:-\${ROS_DISTRO}}-ros-dev-tools' "${ROS_INSTALLER}" && \
   ! grep -Fq 'ros-${ROS_DISTRO}-ros-dev-tools' "${ROS_INSTALLER}"; then
  echo "install_ros2.sh must install ros-<distro>-ros-dev-tools alongside ros-base." >&2
  exit 1
fi

if ! grep -Fq 'export ROS_DISTRO=${ROS_DISTRO}' "${ROS_INSTALLER}"; then
  echo "install_ros2.sh must export ROS_DISTRO in the default profile to support module provisioning." >&2
  exit 1
fi

if grep -Fq 'python3-colcon' "${ROS_INSTALLER}"; then
  echo "install_ros2.sh must not install python3-colcon-* packages." >&2
  exit 1
fi
