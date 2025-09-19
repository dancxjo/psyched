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

# Engine setup
ENGINE="${VOICE_ENGINE:-piper}" # piper | espeak

if [[ "${ENGINE}" == "piper" ]]; then
  # Download and setup Piper voice model
  VOICE_DIR="${REPO_DIR}/voices"
  VOICE_MODEL="${VOICE_DIR}/en_US-john-medium.onnx"
  VOICE_URL="https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/john/medium/en_US-john-medium.onnx"

  mkdir -p "${VOICE_DIR}"
  if [ ! -f "${VOICE_MODEL}" ]; then
    echo "Downloading Piper voice model..."
    curl -L -o "${VOICE_MODEL}" "${VOICE_URL}"
    echo "Voice model downloaded to ${VOICE_MODEL}"
  fi
  if [ -z "${PIPER_VOICE:-}" ]; then
    export PIPER_VOICE="${VOICE_MODEL}"
    echo "PIPER_VOICE set to ${VOICE_MODEL}"
  else
    echo "PIPER_VOICE already set to ${PIPER_VOICE}"
  fi

  # Python library is optional but helpful for fallback
  if ! python3 -c 'import piper' >/dev/null 2>&1; then
    echo "Installing optional piper-tts library (for fallback)..."
    if pip3 install piper-tts >/dev/null 2>&1 || sudo pip3 install piper-tts >/dev/null 2>&1; then
      echo "Installed piper-tts"
    else
      echo "Skipping piper-tts install"
    fi
  fi
else
  # espeak-ng + MBROLA setup
  echo "Configuring espeak-ng engine..."
  # Try to install packages if not present
  if ! command -v espeak-ng >/dev/null 2>&1 && ! command -v espeak >/dev/null 2>&1; then
    echo "espeak-ng not found. Attempting to install..."
    if command -v apt >/dev/null 2>&1; then
      sudo apt update && sudo apt install -y espeak-ng mbrola || true
    else
      echo "Non-APT system; please install espeak-ng and MBROLA voices manually."
    fi
  fi
  # Suggest environment variable for voice
  if [ -z "${ESPEAK_VOICE:-}" ]; then
    export ESPEAK_VOICE="mb-en1"
    echo "ESPEAK_VOICE set to ${ESPEAK_VOICE}"
  fi
  # Attempt to install the requested MBROLA voice if on apt-based systems
  if command -v apt >/dev/null 2>&1; then
    # Common package name for English MBROLA voice
    sudo apt install -y mbrola-en1 || true
  fi
fi

# Install fortune-mod and some extra fortunes if available
if command -v apt >/dev/null 2>&1; then
  sudo apt install -y fortune-mod fortunes fortunes-min || true
fi

echo "Voice module setup complete. ENGINE=${ENGINE}"
