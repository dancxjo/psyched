#!/usr/bin/env bash
set -euo pipefail

# Ear module setup: link package(s) into src/ and ensure ALSA utils.

REPO_DIR="$(pwd)"
SRC_DIR="${REPO_DIR}/src"
PKG_DIR="${REPO_DIR}/packages"

mkdir -p "${SRC_DIR}" "${PKG_DIR}"

# Clean current src to ensure only desired packages are linked
find "${SRC_DIR}" -mindepth 1 -maxdepth 1 -exec rm -rf {} +

# Optionally include core psyched package too
if [ -d "${PKG_DIR}/psyched" ]; then
  ln -sfn "${PKG_DIR}/psyched" "${SRC_DIR}/psyched"
fi

# Link the ear package
ln -sfn "${PKG_DIR}/ear" "${SRC_DIR}/ear"

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
