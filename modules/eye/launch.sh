#!/usr/bin/env bash
set -euo pipefail

# Source ROS environment if available, disable nounset to avoid unbound variable errors
if [ -f "/opt/ros/${ROS_DISTRO:-kilted}/setup.bash" ]; then
  set +u
  source "/opt/ros/${ROS_DISTRO:-kilted}/setup.bash"
  set -u
fi
if [ -f "install/setup.bash" ]; then
  set +u
  source "install/setup.bash"
  set -u
fi

# Try to run the node via ros2 run first
if ros2 run kinect_ros2 kinect_ros2_node 2>/dev/null; then
  exit 0
fi

echo "[eye/launch] Falling back to running kinect_ros2_node directly (if available)."
if command -v kinect_ros2_node >/dev/null 2>&1; then
  exec kinect_ros2_node
else
  echo "[eye/launch] Could not find kinect_ros2_node. Ensure workspace is built and sourced." >&2
  exit 1
fi
