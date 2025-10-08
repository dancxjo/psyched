#!/usr/bin/env bash
set -euo pipefail

source_setup_script() {
  local script_path="$1"
  if [ -f "$script_path" ]; then
    # Many ROS setup scripts probe optional variables such as AMENT_TRACE_SETUP_FILES
    # without defining them first. Temporarily relax nounset so we can source them
    # safely while still restoring the stricter shell options afterwards.
    set +u
    export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES-}"
    # shellcheck disable=SC1090
    source "$script_path"
    set -u
  fi
}

if [ "${BUILD:-true}" = "true" ]; then
  ROS_DISTRO=${ROS_DISTRO:-humble}
  source_setup_script "/opt/ros/${ROS_DISTRO}/setup.bash"
  source_setup_script "install/setup.bash"
  if command -v colcon >/dev/null 2>&1; then
    colcon build --packages-select psyched_nav --symlink-install || true
  fi
fi
