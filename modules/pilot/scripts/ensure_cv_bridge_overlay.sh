#!/usr/bin/env bash
set -euo pipefail

# Ensure the Kinect driver builds against cv_bridge by overlaying the sources
# when the binary headers are unavailable and by wiring the targets to link to
# the library.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
SRC_DIR="${PSYCHED_WORKSPACE_SRC:-${REPO_ROOT}/work/src}"
ROS_DISTRO="${ROS_DISTRO:-kilted}"
OVERLAY_DIR="${SRC_DIR}/vision_opencv"
HEADER="/opt/ros/${ROS_DISTRO}/include/cv_bridge/cv_bridge.hpp"
CMAKE_FILE="${SRC_DIR}/kinect_ros2/CMakeLists.txt"

log() {
  echo "[pilot/cv_bridge] $*"
}

if [[ ! -d "${SRC_DIR}/kinect_ros2" ]]; then
  log "kinect_ros2 package not present; skipping overlay"
  exit 0
fi

try_install_apt_cv_bridge() {
  if ! command -v apt-get >/dev/null 2>&1; then
    return 1
  fi

  local sudo_cmd=""
  if command -v sudo >/dev/null 2>&1 && [[ ${EUID:-0} -ne 0 ]]; then
    sudo_cmd="sudo"
  fi

  local run_apt=()
  if [[ -n "${sudo_cmd}" ]]; then
    run_apt=("${sudo_cmd}" apt-get)
  else
    run_apt=(apt-get)
  fi

  "${run_apt[@]}" update -y >/dev/null 2>&1 || true
  if "${run_apt[@]}" install -y --no-install-recommends "ros-${ROS_DISTRO}-vision-opencv" >/dev/null 2>&1; then
    return 0
  fi

  "${run_apt[@]}" install -y --no-install-recommends \
    "ros-${ROS_DISTRO}-cv-bridge" \
    "ros-${ROS_DISTRO}-image-geometry" >/dev/null 2>&1 || return 1

  return 0
}

ensure_cv_bridge_sources() {
  if [[ -f "${HEADER}" ]]; then
    if [[ -d "${OVERLAY_DIR}" ]]; then
      rm -rf "${OVERLAY_DIR}"
    fi
    return 0
  fi

  if try_install_apt_cv_bridge && [[ -f "${HEADER}" ]]; then
    if [[ -d "${OVERLAY_DIR}" ]]; then
      rm -rf "${OVERLAY_DIR}"
    fi
    return 0
  fi

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
}

ensure_cv_bridge_sources

if [[ ! -f "${CMAKE_FILE}" ]]; then
  log "CMakeLists.txt missing at ${CMAKE_FILE}; skipping build patch"
  exit 0
fi

if ! grep -Eq 'find_package\(cv_bridge' "${CMAKE_FILE}"; then
  if grep -n 'find_package(OpenCV' "${CMAKE_FILE}" >/dev/null; then
    line=$(grep -n 'find_package(OpenCV' "${CMAKE_FILE}" | head -n1 | cut -d: -f1)
    awk -v ln="${line}" 'NR==ln{print; print "find_package(cv_bridge REQUIRED)"; next}1' "${CMAKE_FILE}" >"${CMAKE_FILE}.tmp"
    mv "${CMAKE_FILE}.tmp" "${CMAKE_FILE}"
    log "Injected cv_bridge find_package next to OpenCV stanza"
  else
    printf '\nfind_package(cv_bridge REQUIRED)\n' >>"${CMAKE_FILE}"
    log "Appended cv_bridge find_package"
  fi
fi

ensure_target_links_with_cv_bridge() {
  local target="$1"
  if grep -E "target_link_libraries *\\(${target}" "${CMAKE_FILE}" >/dev/null; then
    if ! sed -n "/target_link_libraries\\(${target}/,/)/p" "${CMAKE_FILE}" | grep -q "cv_bridge"; then
      awk -v tgt="target_link_libraries(${target}" '
        BEGIN{in_block=0}
        index($0,tgt){print;in_block=1;next}
        in_block&&/\)/{print "    cv_bridge::cv_bridge";print;in_block=0;next}
        {print}
      ' "${CMAKE_FILE}" >"${CMAKE_FILE}.tmp"
      mv "${CMAKE_FILE}.tmp" "${CMAKE_FILE}"
      log "Linked ${target} against cv_bridge target"
    fi
  elif grep -E "ament_target_dependencies *\\(${target}" "${CMAKE_FILE}" >/dev/null; then
    if ! sed -n "/ament_target_dependencies\\(${target}/,/)/p" "${CMAKE_FILE}" | grep -q "cv_bridge"; then
      awk -v tgt="ament_target_dependencies(${target}" '
        BEGIN{in_block=0}
        index($0,tgt){print;in_block=1;next}
        in_block&&/\)/{print "  cv_bridge";print;in_block=0;next}
        {print}
      ' "${CMAKE_FILE}" >"${CMAKE_FILE}.tmp"
      mv "${CMAKE_FILE}.tmp" "${CMAKE_FILE}"
      log "Added cv_bridge to ament_target_dependencies for ${target}"
    fi
  fi
}

for target in kinect_ros2_component kinect_ros2_node; do
  ensure_target_links_with_cv_bridge "${target}"
done

find "${SRC_DIR}" -type f \( -name '*.cpp' -o -name '*.hpp' -o -name '*.h' -o -name '*.cc' -o -name '*.cxx' \) | while read -r file; do
  if grep -Eq 'cv_bridge/cv_bridge\.h([^p]|$)|cv_bridge/cv_bridge\.hppp' "${file}"; then
    python3 - "$file" <<'PY'
import pathlib
import re
import sys

path = pathlib.Path(sys.argv[1])
text = path.read_text()
normalised = re.sub(r"cv_bridge/cv_bridge\.hppp+", "cv_bridge/cv_bridge.hpp", text)
normalised = re.sub(r"cv_bridge/cv_bridge\.h(?=[^p]|$)", "cv_bridge/cv_bridge.hpp", normalised)
if normalised != text:
    path.write_text(normalised)
PY
    log "Normalised cv_bridge headers in ${file}"
  fi
done

log "cv_bridge overlay ready"
