#!/usr/bin/env bash
set -euo pipefail

# Config: source ../../config/ear.env from real script location
REAL_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$REAL_PATH")"
MODULE_NAME="$(basename "$(dirname "$REAL_PATH")")"
CONF_FILE="$(cd "$SCRIPT_DIR/../.." && pwd)/config/${MODULE_NAME}.env"
if [ -f "$CONF_FILE" ]; then
  # shellcheck disable=SC1090
  . "$CONF_FILE"
fi


# Ear module setup: link package(s) into src/ and ensure ALSA utils.

case "${ROS_DISTRO}" in
  jade|kinetic|lunar|melodic|noetic)
    echo "Skipping end-of-life ROS distro: ${ROS_DISTRO}"
    exit 0
    ;;
  jazzy|kilted|rolling)
    echo "Detected supported ROS2 distro: ${ROS_DISTRO}"
    ;;
  *)
    echo "Proceeding with generic ROS2 distro: ${ROS_DISTRO}"
    ;;
esac
# Determine repository root regardless of current working directory
if REPO_DIR_GIT_ROOT=$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel 2>/dev/null); then
  REPO_DIR="$REPO_DIR_GIT_ROOT"
else
  REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
fi
SRC_DIR="${REPO_DIR}/src"
# Packages are now located under modules/<module>/packages/<module>
PKG_DIR="${REPO_DIR}/modules/${MODULE_NAME}/packages"

mkdir -p "${SRC_DIR}" "${PKG_DIR}"

# Link the module package into src/ (e.g. modules/ear/packages/ear -> src/ear)
if [ -d "${PKG_DIR}/${MODULE_NAME}" ]; then
  ln -sfn "${PKG_DIR}/${MODULE_NAME}" "${SRC_DIR}/${MODULE_NAME}"
else
  echo "[ear/setup] Warning: module-local package not found: ${PKG_DIR}/${MODULE_NAME}" >&2
fi

# Link module-local psyced_msgs if present (named psyched_msgs in repo)
if [ -d "${PKG_DIR}/psyched_msgs" ]; then
  ln -sfn "${PKG_DIR}/psyched_msgs" "${SRC_DIR}/psyched_msgs"
else
  echo "[ear/setup] Warning: module-local package not found: ${PKG_DIR}/psyched_msgs" >&2
fi

# Install Python dependencies for ear module
echo "[ear/setup] Installing Python dependencies..."
if ! python3 -c 'import pyaudio' >/dev/null 2>&1; then
  echo "[ear/setup] Installing pyaudio library..."
  if pip3 install --break-system-packages pyaudio >/dev/null 2>&1 || sudo pip3 install --break-system-packages pyaudio >/dev/null 2>&1; then
    echo "[ear/setup] Installed pyaudio"
  else
    echo "[ear/setup] Warning: Failed to install pyaudio; ear module may not function properly" >&2
  fi
else
  echo "[ear/setup] pyaudio already present"
fi

if ! python3 -c 'import webrtcvad' >/dev/null 2>&1; then
  echo "[ear/setup] Installing webrtcvad library..."
  if pip3 install --break-system-packages webrtcvad >/dev/null 2>&1 || sudo pip3 install --break-system-packages webrtcvad >/dev/null 2>&1; then
    echo "[ear/setup] Installed webrtcvad"
  else
    echo "[ear/setup] Warning: Failed to install webrtcvad; VAD functionality may not work properly" >&2
  fi
else
  echo "[ear/setup] webrtcvad already present"
fi

# Ensure arecord is available
if command -v apt >/dev/null 2>&1; then
  sudo apt update && sudo apt install -y alsa-utils || true
fi

echo "Ear module setup complete."
