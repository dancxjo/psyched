#!/usr/bin/env bash
set -euo pipefail

# Pilot module setup: build local packages using a fresh src/ populated by symlinks.

REPO_DIR="$(pwd)"
SRC_DIR="${REPO_DIR}/src"
PKG_DIR="${REPO_DIR}/packages"

mkdir -p "${SRC_DIR}" "${PKG_DIR}"

# Clean current src to ensure only desired packages are linked
find "${SRC_DIR}" -mindepth 1 -maxdepth 1 -exec rm -rf {} +

# Link the packages we want in this module
ln -sfn "${PKG_DIR}/pilot" "${SRC_DIR}/pilot"

# Include geometry_msgs dependency (part of ROS2 common interfaces)
# and any other core dependencies needed

# Check if websockets library is available
echo "[pilot/setup] Checking Python dependencies..."
if ! python3 -c 'import websockets' >/dev/null 2>&1; then
  echo "[pilot/setup] Installing websockets library..."
  if pip3 install websockets >/dev/null 2>&1 || sudo pip3 install websockets >/dev/null 2>&1; then
    echo "[pilot/setup] websockets installed successfully"
  else
    echo "[pilot/setup] Warning: Failed to install websockets. You may need to install it manually:"
    echo "[pilot/setup]   pip install websockets"
  fi
else
  echo "[pilot/setup] websockets library already available"
fi

echo "[pilot/setup] Done. Build with: make build"