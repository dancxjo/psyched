#!/usr/bin/env bash
set -euo pipefail

# Voice module setup: build local packages using a fresh src/ populated by symlinks.

REPO_DIR="$(pwd)"
SRC_DIR="${REPO_DIR}/src"
PKG_DIR="${REPO_DIR}/packages"

mkdir -p "${SRC_DIR}" "${PKG_DIR}"

# Clean current src to ensure only desired packages are linked
find "${SRC_DIR}" -mindepth 1 -maxdepth 1 -exec rm -rf {} +

# Link the packages we want in this module
ln -sfn "${PKG_DIR}/voice" "${SRC_DIR}/voice"

# Optionally include core psyched package too
if [ -d "${PKG_DIR}/psyched" ]; then
  ln -sfn "${PKG_DIR}/psyched" "${SRC_DIR}/psyched"
fi

# Ensure Piper voice model path is pointed via env var if available
if [ -z "${PIPER_VOICE:-}" ]; then
  echo "Note: PIPER_VOICE env var not set. Set it to a .onnx voice file for TTS."
fi

# Install runtime deps that rosdep won't handle (Python pip packages)
if ! python3 -c 'import piper' >/dev/null 2>&1; then
  pip3 install --user piper-tts
fi

echo "Voice module setup complete."
