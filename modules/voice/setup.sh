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

# Download and setup voice model
VOICE_DIR="${REPO_DIR}/voices"
VOICE_MODEL="${VOICE_DIR}/en_US-john-medium.onnx"
VOICE_URL="https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/john/medium/en_US-john-medium.onnx"

mkdir -p "${VOICE_DIR}"

# Download voice model if it doesn't exist
if [ ! -f "${VOICE_MODEL}" ]; then
  echo "Downloading Piper voice model..."
  curl -L -o "${VOICE_MODEL}" "${VOICE_URL}"
  echo "Voice model downloaded to ${VOICE_MODEL}"
fi

# Set PIPER_VOICE environment variable if not already set
if [ -z "${PIPER_VOICE:-}" ]; then
  export PIPER_VOICE="${VOICE_MODEL}"
  echo "PIPER_VOICE set to ${VOICE_MODEL}"
else
  echo "PIPER_VOICE already set to ${PIPER_VOICE}"
fi

# Install runtime deps that rosdep won't handle (Python pip packages)
if ! python3 -c 'import piper' >/dev/null 2>&1; then
  pip3 install --user piper-tts
fi

echo "Voice module setup complete."
