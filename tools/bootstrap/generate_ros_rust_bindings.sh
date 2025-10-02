#!/usr/bin/env bash
#
# generate_ros_rust_bindings.sh - Build ROS 2 Rust message crates inside Docker and
# vendor them into the repository so Cargo patches resolve locally.
#
# This script intentionally keeps the host environment lean: we only require Docker
# and the ROS apt repos installed by install_ros2.sh. The heavy lifting happens in a
# throwaway container that installs rosidl_generator_rs, builds the desired message
# packages with colcon, and copies their generated `rust/` crates into vendor_msgs/.
#
# Usage:
#   ./generate_ros_rust_bindings.sh        # uses ROS_DISTRO (defaults to kilted)
#   ROS_DISTRO=jazzy ./generate_ros_rust_bindings.sh
#
# The script is idempotent: it clears any previous vendor_msgs/<pkg> directory before
# copying in the freshly generated crate. If Docker is unavailable, the script emits
# a warning and exits successfully so provisioning can continue.

set -euo pipefail

ROS_DISTRO=${ROS_DISTRO:-kilted}
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "${SCRIPT_DIR}/../.." && pwd)
VENDOR_DIR="${REPO_ROOT}/vendor_msgs"
PACKAGES=(
  actionlib_msgs
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  shape_msgs
  std_msgs
  trajectory_msgs
)

log() {
  printf '[ros-rust-bindings] %s\n' "$*"
}

if ! command -v docker >/dev/null 2>&1; then
  log "Docker is not available; skipping ROS Rust binding generation."
  log "Install Docker and re-run this script to vendor std_msgs and related crates."
  exit 0
fi

mkdir -p "${VENDOR_DIR}"

BUILD_CONTEXT=$(mktemp -d)
OUTPUT_DIR=$(mktemp -d)
trap 'sudo rm -rf "${BUILD_CONTEXT}" "${OUTPUT_DIR}"' EXIT

cat <<'DOCKERFILE' >"${BUILD_CONTEXT}/Dockerfile"
ARG ROS_DISTRO=kilted
FROM ros:${ROS_DISTRO}-ros-base
ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
      git \
      curl \
      unzip \
      build-essential \
      python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Install Rust toolchain for rosidl_generator_rs (minimal profile).
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --profile minimal
ENV PATH=/root/.cargo/bin:${PATH}

WORKDIR /root/ws/src
RUN curl -L https://github.com/ros2/common_interfaces/archive/refs/heads/rolling.zip -o common_interfaces.zip \
    && unzip common_interfaces.zip \
    && mv common_interfaces-rolling common_interfaces \
    && rm common_interfaces.zip \
    && curl -L https://github.com/ros2-rust/rosidl_rust/archive/refs/heads/main.zip -o rosidl_rust.zip \
    && unzip rosidl_rust.zip \
    && mv rosidl_rust-main rosidl_rust \
    && rm rosidl_rust.zip

WORKDIR /root/ws
RUN bash -lc 'source /opt/ros/${ROS_DISTRO}/setup.bash \
  && colcon build --symlink-install --merge-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release'
DOCKERFILE

IMAGE_TAG="psyched/ros-rust-bindings:${ROS_DISTRO}"
log "Building Docker image ${IMAGE_TAG} with ROS_DISTRO=${ROS_DISTRO}..."
docker build --build-arg ROS_DISTRO="${ROS_DISTRO}" -t "${IMAGE_TAG}" "${BUILD_CONTEXT}"

log "Extracting generated Rust crates for ${PACKAGES[*]}..."
docker run --rm \
  -e ROS_DISTRO="${ROS_DISTRO}" \
  -v "${OUTPUT_DIR}:/out" \
  "${IMAGE_TAG}" \
  bash -lc 'set -euo pipefail; \
    shopt -s nullglob; \
    for rust_dir in /root/ws/install/share/*/rust; do \
      pkg=$(basename "$(dirname "${rust_dir}")"); \
      mkdir -p "/out/${pkg}"; \
      cp -rL "${rust_dir}/." "/out/${pkg}/"; \
    done'

log "Copying crates into ${VENDOR_DIR}..."
for pkg in "${PACKAGES[@]}"; do
  src_pkg_dir="${OUTPUT_DIR}/${pkg}"
  if [[ ! -d "${src_pkg_dir}" ]]; then
    log "Warning: package ${pkg} was not produced by the container build."
    continue
  fi
  rm -rf "${VENDOR_DIR}/${pkg}"
  mkdir -p "${VENDOR_DIR}/${pkg}"
  cp -R "${src_pkg_dir}/." "${VENDOR_DIR}/${pkg}/"
  log "Vendored ${pkg} into vendor_msgs/${pkg}."
done

log "ROS Rust bindings refreshed successfully."
