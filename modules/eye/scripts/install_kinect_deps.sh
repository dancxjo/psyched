#!/usr/bin/env bash
set -euo pipefail

# Install ROS 2 and system dependencies required by the Kinect driver.
if ! command -v apt-get >/dev/null 2>&1; then
  echo "[eye/deps] apt-get not available; skipping dependency installation"
  exit 0
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
SRC_DIR="${PSYCHED_WORKSPACE_SRC:-${REPO_ROOT}/work/src}"
ENSURE_OVERLAY_SCRIPT="${REPO_ROOT}/modules/cockpit/scripts/ensure_cv_bridge_overlay.sh"

ROS_DISTRO=${ROS_DISTRO:-kilted}
HEADER="/opt/ros/${ROS_DISTRO}/include/cv_bridge/cv_bridge.hpp"

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

PACKAGES=(
  "ros-${ROS_DISTRO}-camera-calibration-parsers"
  "ros-${ROS_DISTRO}-image-transport"
  "ros-${ROS_DISTRO}-image-transport-plugins"
  "ros-${ROS_DISTRO}-image-pipeline"
  "ros-${ROS_DISTRO}-perception"
  "ros-${ROS_DISTRO}-perception-pcl"
  "ros-${ROS_DISTRO}-vision-msgs"
  "ros-${ROS_DISTRO}-usb-cam"
  "libogre-1.12-dev"
  "ros-${ROS_DISTRO}-rviz2"
  "libgl1"
  "libegl1"
  "libxrandr2"
  "libxrandr-dev"
  "libxinerama1"
  "libxinerama-dev"
  "libxcursor1"
  "libxcursor-dev"
)

run_apt apt-get install -y "${PACKAGES[@]}" || true

ensure_cv_bridge_via_apt() {
  if run_apt apt-get install -y --no-install-recommends "ros-${ROS_DISTRO}-vision-opencv"; then
    return 0
  fi

  if run_apt apt-get install -y --no-install-recommends \
    "ros-${ROS_DISTRO}-cv-bridge" \
    "ros-${ROS_DISTRO}-image-geometry"; then
    return 0
  fi

  return 1
}

if ! ensure_cv_bridge_via_apt; then
  echo "[eye/deps] Warning: Failed to install cv_bridge via apt packages." >&2
fi

if [[ ! -f "${HEADER}" ]]; then
  echo "[eye/deps] cv_bridge headers missing; attempting overlay helper" >&2
  if [[ -x "${ENSURE_OVERLAY_SCRIPT}" ]]; then
    PSYCHED_WORKSPACE_SRC="${SRC_DIR}" "${ENSURE_OVERLAY_SCRIPT}" || true
  else
    echo "[eye/deps] Overlay helper not found at ${ENSURE_OVERLAY_SCRIPT}" >&2
  fi
fi

if [[ ! -f "${HEADER}" ]]; then
  echo "[eye/deps] cv_bridge headers still unavailable after overlay attempt." >&2
  echo "[eye/deps] You may need to resolve held packages or build cv_bridge from source." >&2
fi
