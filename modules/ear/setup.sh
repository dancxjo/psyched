#!/usr/bin/env bash
set -euo pipefail

# Ear module setup: link package(s) into src/ and ensure ALSA utils.

REPO_DIR="$(pwd)"
SRC_DIR="${REPO_DIR}/src"
PKG_DIR="${REPO_DIR}/packages"

mkdir -p "${SRC_DIR}" "${PKG_DIR}"

# Clean current src to ensure only desired packages are linked
find "${SRC_DIR}" -mindepth 1 -maxdepth 1 -exec rm -rf {} +

# Link the packages we want in this module
ln -sfn "${PKG_DIR}/ear" "${SRC_DIR}/ear"

# Clone audio_common_msgs if not already present
if [ ! -d "${SRC_DIR}/audio_common_msgs" ]; then
    echo "Cloning audio_common_msgs dependency..."
    cd "${SRC_DIR}"
    git clone https://github.com/ros2/audio_common_msgs.git -b kilted || \
    git clone https://github.com/ros2/audio_common_msgs.git -b main || \
    git clone https://github.com/ros2/audio_common_msgs.git
    cd "${REPO_DIR}"
fi

# Optionally include core psyched package too
if [ -d "${PKG_DIR}/psyched" ]; then
  ln -sfn "${PKG_DIR}/psyched" "${SRC_DIR}/psyched"
fi

# Ensure arecord is available
if command -v apt >/dev/null 2>&1; then
  sudo apt update && sudo apt install -y alsa-utils || true
fi

echo "Ear module setup complete."
