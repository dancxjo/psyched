#!/usr/bin/env bash
#
# setup_ros_rust_bindings.sh - Clone the ros2-rust repository needed for generating
# ROS 2 Rust message crates.
#
# This script is idempotent: it removes any previous checkout before cloning.

set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "${SCRIPT_DIR}/../.." && pwd)
ROS_RUST_SRC_DIR="${REPO_ROOT}/work/src/rosidl_rust"

log() {
  printf '[ros-rust-setup] %s\n' "$*"
}

log "Cloning ros2-rust/rosidl_rust into ${ROS_RUST_SRC_DIR}..."
rm -rf "${ROS_RUST_SRC_DIR}"
mkdir -p "${ROS_RUST_SRC_DIR}"
git clone https://github.com/ros2-rust/rosidl_rust.git "${ROS_RUST_SRC_DIR}"
log "ros2-rust/rosidl_rust cloned successfully."
