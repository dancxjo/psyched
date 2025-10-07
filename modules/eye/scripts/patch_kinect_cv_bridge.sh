#!/usr/bin/env bash
set -euo pipefail

# Ensure the Kinect driver links against cv_bridge when available.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
SRC_DIR="${PSYCHED_WORKSPACE_SRC:-${REPO_ROOT}/work/src}"
CMAKE_FILE="${SRC_DIR}/kinect_ros2/CMakeLists.txt"

if [[ ! -f "${CMAKE_FILE}" ]]; then
  echo "[eye/patch] CMakeLists.txt not found at ${CMAKE_FILE}; skipping"
  exit 0
fi

if ! grep -Eq 'find_package\(cv_bridge' "${CMAKE_FILE}"; then
  if grep -n 'find_package(OpenCV' "${CMAKE_FILE}" >/dev/null; then
    line=$(grep -n 'find_package(OpenCV' "${CMAKE_FILE}" | head -n1 | cut -d: -f1)
    awk -v ln="${line}" 'NR==ln{print; print "find_package(cv_bridge REQUIRED)"; next}1' "${CMAKE_FILE}" >"${CMAKE_FILE}.tmp"
    mv "${CMAKE_FILE}.tmp" "${CMAKE_FILE}"
  else
    printf '\nfind_package(cv_bridge REQUIRED)\n' >>"${CMAKE_FILE}"
  fi
fi

for target in kinect_ros2_component kinect_ros2_node; do
  if ! grep -E "ament_target_dependencies\(${target}" "${CMAKE_FILE}" | grep -q cv_bridge; then
    awk -v tgt="ament_target_dependencies(${target}" '
      BEGIN{in_block=0}
      index($0,tgt){print;in_block=1;next}
      in_block&&/\)/{print "  cv_bridge";print;in_block=0;next}
      {print}
    ' "${CMAKE_FILE}" >"${CMAKE_FILE}.tmp"
    mv "${CMAKE_FILE}.tmp" "${CMAKE_FILE}"
  fi
done

HEADER_FILE="${SRC_DIR}/kinect_ros2/include/kinect_ros2/kinect_ros2_component.hpp"
if [[ -f "${HEADER_FILE}" ]] && grep -q 'cv_bridge/cv_bridge\.h' "${HEADER_FILE}"; then
  python3 - "${HEADER_FILE}" <<'PY'
import pathlib
import sys

path = pathlib.Path(sys.argv[1])
text = path.read_text()
updated = text.replace('cv_bridge/cv_bridge.h', 'cv_bridge/cv_bridge.hpp')
if updated != text:
    path.write_text(updated)
PY
  echo "[eye/patch] Updated cv_bridge header include to .hpp in ${HEADER_FILE}"
fi

echo "[eye/patch] Ensured cv_bridge dependency in ${CMAKE_FILE}"
