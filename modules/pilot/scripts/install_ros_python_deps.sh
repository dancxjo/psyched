#!/usr/bin/env bash
# Ensure ROS Python shims like rosidl_runtime_py are installed via apt so they
# remain in sync with the system ROS distribution. PIP does not publish these
# packages which caused previous setup attempts to fail.

set -euo pipefail

detect_ros_distro() {
  if [[ -n "${ROS_DISTRO:-}" ]]; then
    echo "${ROS_DISTRO}"
    return 0
  fi
  if [[ -d /opt/ros ]]; then
    for entry in /opt/ros/*; do
      [[ -d "${entry}" ]] || continue
      local setup="${entry}/setup.bash"
      if [[ -f "${setup}" ]]; then
        echo "$(basename "${entry}")"
        return 0
      fi
    done
  fi
  echo "kilted"
  return 0
}

ROS_DISTRO_DETECTED="$(detect_ros_distro)"
PACKAGE="ros-${ROS_DISTRO_DETECTED}-rosidl-runtime-py"

echo "[pilot] Ensuring ${PACKAGE} is installed via apt"

if dpkg -s "${PACKAGE}" >/dev/null 2>&1; then
  echo "[pilot] ${PACKAGE} already present"
  exit 0
fi

if command -v sudo >/dev/null 2>&1 && [[ $(id -u) -ne 0 ]]; then
  SUDO=(sudo)
else
  SUDO=()
fi

"${SUDO[@]}" apt-get update
"${SUDO[@]}" apt-get install -y "${PACKAGE}"
