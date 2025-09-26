#!/bin/bash
set -euo pipefail
CMK="src/kinect_ros2/CMakeLists.txt"
if [ -f "$CMK" ]; then
  if ! grep -E 'find_package\(cv_bridge' "$CMK" >/dev/null 2>&1; then
    if grep -n 'find_package(OpenCV' "$CMK" >/dev/null 2>&1; then
      ln=$(grep -n 'find_package(OpenCV' "$CMK" | head -n1 | cut -d: -f1)
      awk -v ln="$ln" 'NR==ln{print; print "find_package(cv_bridge REQUIRED)"; next}1' "$CMK" >"$CMK.tmp" && mv "$CMK.tmp" "$CMK"
    else
      printf '\nfind_package(cv_bridge REQUIRED)\n' >>"$CMK"
    fi
  fi
  for target in kinect_ros2_component kinect_ros2_node; do
    if ! grep -E "ament_target_dependencies\($target" "$CMK" | grep -q cv_bridge; then
      awk -v tgt="ament_target_dependencies($target" 'BEGIN{inb=0} index($0,tgt){print;inb=1;next} inb&&/\)/{print "  cv_bridge";print;inb=0;next}1' "$CMK" >"$CMK.tmp" && mv "$CMK.tmp" "$CMK"
    fi
  done
fi
