#!/usr/bin/env bash
set -euo pipefail

# Normalize cv_bridge header usage so the Kinect driver builds against ROS binaries.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
SRC_DIR="${PSYCHED_WORKSPACE_SRC:-${REPO_ROOT}/work/src}"
ROS_DISTRO=${ROS_DISTRO:-kilted}
HEADER="/opt/ros/${ROS_DISTRO}/include/cv_bridge/cv_bridge.hpp"
OVERLAY_DIR="${SRC_DIR}/vision_opencv"

ensure_apt_cv_bridge() {
  if ! command -v apt-get >/dev/null 2>&1; then
    return 1
  fi

  local sudo_cmd=""
  if command -v sudo >/dev/null 2>&1 && [[ ${EUID:-0} -ne 0 ]]; then
    sudo_cmd="sudo"
  fi

  local run_apt
  run_apt() {
    if [[ -n ${sudo_cmd} ]]; then
      ${sudo_cmd} "$@"
    else
      "$@"
    fi
  }

  run_apt apt-get update -y >/dev/null 2>&1 || true
  if run_apt apt-get install -y --no-install-recommends \
      "ros-${ROS_DISTRO}-vision-opencv" >/dev/null 2>&1; then
    return 0
  fi

  run_apt apt-get install -y --no-install-recommends \
    "ros-${ROS_DISTRO}-cv-bridge" \
    "ros-${ROS_DISTRO}-image-geometry" >/dev/null 2>&1 || return 1
  return 0
}

if [[ -f "${HEADER}" ]]; then
  if [[ -d "${OVERLAY_DIR}" ]]; then
    rm -rf "${OVERLAY_DIR}"
  fi
else
  if ensure_apt_cv_bridge && [[ -f "${HEADER}" ]]; then
    if [[ -d "${OVERLAY_DIR}" ]]; then
      rm -rf "${OVERLAY_DIR}"
    fi
  else
    if [[ ! -d "${OVERLAY_DIR}/.git" ]]; then
      git clone https://github.com/ros-perception/vision_opencv.git "${OVERLAY_DIR}" >/dev/null 2>&1 || true
    fi
    if [[ -d "${OVERLAY_DIR}/.git" ]]; then
      git -C "${OVERLAY_DIR}" fetch --tags --force >/dev/null 2>&1 || true
      if git -C "${OVERLAY_DIR}" rev-parse --verify "${ROS_DISTRO}" >/dev/null 2>&1; then
        git -C "${OVERLAY_DIR}" checkout "${ROS_DISTRO}" >/dev/null 2>&1 || true
      elif git -C "${OVERLAY_DIR}" rev-parse --verify ros2 >/dev/null 2>&1; then
        git -C "${OVERLAY_DIR}" checkout ros2 >/dev/null 2>&1 || true
      fi
      rm -f "${OVERLAY_DIR}/COLCON_IGNORE" || true
    fi
  fi
fi

# Replace legacy cv_bridge includes with the modern header path.
find "${SRC_DIR}" -type f \( -name '*.cpp' -o -name '*.hpp' -o -name '*.h' -o -name '*.cc' -o -name '*.cxx' \) | while read -r file; do
  if grep -Eq 'cv_bridge/cv_bridge\\.h([^p]|$)|cv_bridge/cv_bridge\\.hppp' "${file}"; then
    python3 - "$file" <<'PY'
import pathlib
import re
import sys

path = pathlib.Path(sys.argv[1])
text = path.read_text()
normalised = re.sub(r"cv_bridge/cv_bridge\\.hppp+", "cv_bridge/cv_bridge.hpp", text)
normalised = re.sub(r"cv_bridge/cv_bridge\\.h(?=[^p]|$)", "cv_bridge/cv_bridge.hpp", normalised)
if normalised != text:
    path.write_text(normalised)
PY
  fi
done

echo "[eye/cv_bridge] cv_bridge headers normalised in ${SRC_DIR}"
