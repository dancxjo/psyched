#!/usr/bin/env bash
set -euo pipefail

# Config: source ../../config/pilot.env from real script location
REAL_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$REAL_PATH")"
MODULE_NAME="$(basename "$(dirname "$REAL_PATH")")"
CONF_FILE="$(cd "$SCRIPT_DIR/../.." && pwd)/config/${MODULE_NAME}.env"
if [ -f "$CONF_FILE" ]; then
  # shellcheck disable=SC1090
  . "$CONF_FILE"
fi


# Pilot module setup: build local packages using a fresh src/ populated by symlinks.

# Determine repository root regardless of current working directory
if REPO_DIR_GIT_ROOT=$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel 2>/dev/null); then
  REPO_DIR="$REPO_DIR_GIT_ROOT"
else
  REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
fi
SRC_DIR="${REPO_DIR}/src"
# Packages located under modules/<module>/packages/<module>
PKG_DIR="${REPO_DIR}/modules/${MODULE_NAME}/packages"

mkdir -p "${SRC_DIR}" "${PKG_DIR}"

# Link the packages we want in this module (modules/pilot/packages/pilot -> src/pilot)
if [ -d "${PKG_DIR}/${MODULE_NAME}" ]; then
  ln -sfn "${PKG_DIR}/${MODULE_NAME}" "${SRC_DIR}/${MODULE_NAME}"
else
  echo "[pilot/setup] Warning: module-local package not found: ${PKG_DIR}/${MODULE_NAME}" >&2
fi

# Include geometry_msgs dependency (part of ROS2 common interfaces)
# and any other core dependencies needed

# Check if websockets library is available
echo "[pilot/setup] Checking Python dependencies..."
if ! python3 -c 'import websockets' >/dev/null 2>&1; then
  echo "[pilot/setup] Installing websockets library (with --break-system-packages)..."
  if pip3 install --break-system-packages websockets >/dev/null 2>&1 || sudo pip3 install --break-system-packages websockets >/dev/null 2>&1; then
    echo "[pilot/setup] websockets installed successfully"
  else
    echo "[pilot/setup] Warning: Failed to install websockets. You may need to install it manually:"
    echo "[pilot/setup]   pip3 install --break-system-packages websockets"
  fi
else
  echo "[pilot/setup] websockets library already available"
fi

echo "[pilot/setup] Done. Build with: make build"