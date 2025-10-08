#!/usr/bin/env bash
# Verifies that the psyched_nav build helper can source ROS setup scripts that
# expect AMENT_TRACE_SETUP_FILES to exist even when the variable is unset.
set -euo pipefail

ros_dir="/opt/ros/testdistro"
setup_file="${ros_dir}/setup.bash"
ros_dir_preexisting=false
if [ -d "$ros_dir" ]; then
  ros_dir_preexisting=true
else
  mkdir -p "$ros_dir"
fi

cleanup() {
  rm -f "$setup_file"
  if [ "$ros_dir_preexisting" = false ]; then
    rmdir "$ros_dir" 2>/dev/null || true
    rmdir "/opt/ros" 2>/dev/null || true
  fi
}
trap cleanup EXIT

cat >"$setup_file" <<'SCRIPT'
if [ "${AMENT_TRACE_SETUP_FILES}" = "1" ]; then
  echo "Tracing"
fi
export TEST_SETUP_RAN=1
SCRIPT

set +u
unset AMENT_TRACE_SETUP_FILES || true
set -u

BUILD=true ROS_DISTRO=testdistro modules/nav/scripts/build_psyched_nav.sh

echo "Setup script sourced successfully"
