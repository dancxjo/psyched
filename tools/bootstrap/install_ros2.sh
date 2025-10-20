#!/usr/bin/env bash
#
# install_ros2.sh - Provision ROS 2 on Debian/Ubuntu using the upstream apt packages.
#
# This script mirrors the `psh dep ros2` command. The ROS distribution can be
# overridden via the ROS_DISTRO environment variable.
#
# Usage examples:
#   ROS_DISTRO=kilted ./install_ros2.sh
#   ROS_DISTRO=rolling ./install_ros2.sh
#   ./install_ros2.sh            # uses ROS_DISTRO (defaults to kilted)
#
# NOTE: Prefer running 'psh host setup' which now provides a Deno-based installer pipeline.
#
set -euo pipefail

ROS_DISTRO=${ROS_DISTRO:-kilted}
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "${SCRIPT_DIR}/../.." && pwd)
ROS_DOMAIN_ID_FILE="${REPO_ROOT}/config/ros_domain_id"

DEFAULT_ROS_DOMAIN_ID="0"
if [[ -f "${ROS_DOMAIN_ID_FILE}" ]]; then
  DEFAULT_ROS_DOMAIN_ID=$(tr -d '[:space:]' < "${ROS_DOMAIN_ID_FILE}")
  if [[ -z "${DEFAULT_ROS_DOMAIN_ID}" ]]; then
    DEFAULT_ROS_DOMAIN_ID="0"
  fi
fi

export ROS_DOMAIN_ID="${DEFAULT_ROS_DOMAIN_ID}"

SUDO=(sudo)
if [[ $(id -u) -eq 0 ]]; then
  SUDO=()
elif ! command -v sudo >/dev/null 2>&1; then
  echo "This script requires root privileges or sudo to escalate." >&2
  exit 1
fi

OS_ID="ubuntu"
OS_ID_LIKE=" "
OS_CODENAME=""
if [[ -r /etc/os-release ]]; then
  # shellcheck disable=SC1091
  . /etc/os-release
  OS_ID="${ID:-${OS_ID}}"
  OS_ID="${OS_ID,,}"
  if [[ -n "${ID_LIKE:-}" ]]; then
    OS_ID_LIKE=" ${ID_LIKE,,} "
  fi
  if [[ -n "${VERSION_CODENAME:-}" ]]; then
    OS_CODENAME="${VERSION_CODENAME,,}"
  fi
fi
if [[ -z "${OS_CODENAME}" ]] && command -v lsb_release >/dev/null 2>&1; then
  OS_CODENAME="$(lsb_release -cs 2>/dev/null | tr '[:upper:]' '[:lower:]' || true)"
fi
if [[ -z "${OS_CODENAME}" ]]; then
  OS_CODENAME="focal"
fi

echo "Detected distribution: ${OS_ID} (${OS_CODENAME})"
echo "Provisioning ROS 2 ${ROS_DISTRO} using Debian packages..."

export LANG=en_US.UTF-8

"${SUDO[@]}" apt-get update
"${SUDO[@]}" apt-get install -y --no-install-recommends \
  ca-certificates \
  curl \
  locales \
  python3-pip \
  python3-venv \
  software-properties-common
"${SUDO[@]}" locale-gen en_US en_US.UTF-8
"${SUDO[@]}" update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

locale

ENABLE_UNIVERSE=0
if [[ "${OS_ID}" == "ubuntu" ]] || [[ "${OS_ID_LIKE}" == *" ubuntu "* ]]; then
  ENABLE_UNIVERSE=1
fi

if [[ "${ENABLE_UNIVERSE}" -eq 1 ]]; then
  "${SUDO[@]}" add-apt-repository -y universe
else
  echo "Skipping Ubuntu 'universe' repository enablement for ${OS_ID}."
fi
"${SUDO[@]}" apt-get update

echo "Setting up ROS 2 apt repository..."

ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')

curl -fsSL -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${OS_CODENAME}_all.deb"

echo "Installing ROS 2 ${ROS_DISTRO} packages..."


# Install only minimal ROS2 base (no desktop/GUI/Qt dependencies)
"${SUDO[@]}" dpkg -i /tmp/ros2-apt-source.deb
"${SUDO[@]}" apt-get update

# Remove the legacy python3-catkin-pkg package if present. The ROS apt
# repository ships python3-catkin-pkg-modules which owns the same files, and
# leaving both installed triggers a dpkg conflict during provisioning.
if dpkg -s python3-catkin-pkg >/dev/null 2>&1; then
  echo "Removing conflicting python3-catkin-pkg package before installing ROS 2 base packages..."
  "${SUDO[@]}" apt-get remove -y python3-catkin-pkg
  "${SUDO[@]}" apt-get install -y -f
  "${SUDO[@]}" dpkg --configure -a
fi

# ros-dev-tools pulls in colcon and other build tooling required by the Psyched stack.
"${SUDO[@]}" apt-get install -y --no-install-recommends \
  ros-${ROS_DISTRO}-ros-base \
  ros-${ROS_DISTRO}-ros-dev-tools \
  python3-rosdep

COLCON_VENV="/opt/ros/${ROS_DISTRO}/colcon-venv"
COLCON_BIN="${COLCON_VENV}/bin/colcon"

echo "Installing colcon into dedicated virtual environment at ${COLCON_VENV}..."
"${SUDO[@]}" mkdir -p "${COLCON_VENV}"
if [[ ! -f "${COLCON_VENV}/pyvenv.cfg" ]]; then
  "${SUDO[@]}" python3 -m venv "${COLCON_VENV}"
fi

if [[ ! -x "${COLCON_VENV}/bin/pip" ]]; then
  "${SUDO[@]}" "${COLCON_VENV}/bin/python" -m ensurepip --upgrade
fi

"${SUDO[@]}" "${COLCON_VENV}/bin/pip" install --no-cache-dir --upgrade pip
"${SUDO[@]}" "${COLCON_VENV}/bin/pip" install --no-cache-dir --upgrade \
  colcon-core \
  colcon-common-extensions

if [[ ! -L /usr/local/bin/colcon ]] || [[ "$(readlink -f /usr/local/bin/colcon 2>/dev/null)" != "${COLCON_BIN}" ]]; then
  "${SUDO[@]}" ln -sf "${COLCON_BIN}" /usr/local/bin/colcon
fi

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  "${SUDO[@]}" rosdep init
fi

# Note: desktop/GUI packages (rviz, Qt, etc.) are intentionally excluded for minimal install.
rosdep update

"${SUDO[@]}" tee /etc/profile.d/ros2-defaults.sh >/dev/null <<PROFILE
# ROS 2 defaults provisioned by install_ros2.sh
export LANG=en_US.UTF-8
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=${DEFAULT_ROS_DOMAIN_ID}
export ROS_LOCALHOST_ONLY=0
export ROS_DISTRO=${ROS_DISTRO}
PROFILE

"${SUDO[@]}" chmod 644 /etc/profile.d/ros2-defaults.sh

ensure_bashrc_sourcing() {
  local target_user="$1" home_dir marker footer bashrc
  if [[ -z "${target_user}" ]]; then
    return 0
  fi
  if ! command -v getent >/dev/null 2>&1; then
    return 0
  fi
  home_dir=$(getent passwd "${target_user}" | awk -F: '{print $6}')
  if [[ -z "${home_dir}" || ! -d "${home_dir}" ]]; then
    return 0
  fi
  bashrc="${home_dir}/.bashrc"
  marker="# >>> ROS 2 ${ROS_DISTRO} environment >>>"
  footer="# <<< ROS 2 ${ROS_DISTRO} environment <<<"
  mkdir -p "${home_dir}"
  touch "${bashrc}"
  if grep -Fq "${marker}" "${bashrc}" >/dev/null 2>&1; then
    return 0
  fi
  {
    printf '\n%s\n' "${marker}"
    printf '%s\n' "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then"
    printf '%s\n' '    # shellcheck disable=SC1091'
    printf '%s\n' "    source /opt/ros/${ROS_DISTRO}/setup.bash"
    printf '%s\n' 'fi'
    printf '%s\n' "${footer}"
  } >> "${bashrc}"
}

ensure_bashrc_sourcing "${SUDO_USER:-${USER:-}}"

echo "ROS 2 ${ROS_DISTRO} installation completed with Cyclone DDS as the default RMW."
echo "Source /opt/ros/${ROS_DISTRO}/setup.bash to begin using ROS 2."
