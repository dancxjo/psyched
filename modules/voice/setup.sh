#!/usr/bin/env bash
set -euo pipefail

# Config: source hosts/<host>/config/voice.env if present relative to repo root, else fallback to ./config/voice.env
REAL_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$REAL_PATH")"          # .../modules/voice
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
HOST_SHORT="${HOST:-$(hostname -s)}"
CONF_FILE=""
if [ -f "${REPO_DIR}/hosts/${HOST_SHORT}/config/voice.env" ]; then
  CONF_FILE="${REPO_DIR}/hosts/${HOST_SHORT}/config/voice.env"
elif [ -f "${REPO_DIR}/hosts/$(hostname -s)/config/voice.env" ]; then
  CONF_FILE="${REPO_DIR}/hosts/$(hostname -s)/config/voice.env"
elif [ -f "${REPO_DIR}/config/voice.env" ]; then
  CONF_FILE="${REPO_DIR}/config/voice.env"
fi
if [ -n "$CONF_FILE" ] && [ -f "$CONF_FILE" ]; then
  # shellcheck disable=SC1090
  . "$CONF_FILE"
fi

# Voice module setup: build local packages using a fresh src/ populated by symlinks.

# Determine repository root regardless of current working directory
if REPO_DIR_GIT_ROOT=$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel 2>/dev/null); then
  REPO_DIR="$REPO_DIR_GIT_ROOT"
else
  REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
fi
SRC_DIR="${REPO_DIR}/src"
# Packages are now located under modules/<module>/packages/<module>
PKG_DIR="${REPO_DIR}/modules/voice/packages"

mkdir -p "${SRC_DIR}" "${PKG_DIR}"

# Link the packages we want in this module (module name -> directory name)
if [ -d "${PKG_DIR}/voice" ]; then
  ln -sfn "${PKG_DIR}/voice" "${SRC_DIR}/voice"
else
  echo "[voice/setup] Warning: module-local package not found: ${PKG_DIR}/voice" >&2
fi

# Link module-local psyched_msgs if present
if [ -d "${PKG_DIR}/psyched_msgs" ]; then
  ln -sfn "${PKG_DIR}/psyched_msgs" "${SRC_DIR}/psyched_msgs"
else
  echo "[voice/setup] Warning: module-local package not found: ${PKG_DIR}/psyched_msgs" >&2
fi

# Engine setup - default to espeak for reliability
ENGINE="${VOICE_ENGINE:-espeak-ng}" # espeak-ng only

echo "[voice/setup] Installing espeak-ng and mbrola-en1..."
sudo apt-get update
sudo apt-get install -y espeak-ng mbrola mbrola-en1

echo "[voice/setup] Installing pyttsx3 for TTS Python support..."
python3 -m pip install --upgrade pip
python3 -m pip install pyttsx3
  if command -v apt >/dev/null 2>&1; then
    # Common package name for English MBROLA voice
    sudo apt install -y mbrola-en1 || true
  fi
fi

# Ensure ffmpeg is present (used by audio processing/playback in voice)
if command -v apt >/dev/null 2>&1; then
  sudo apt-get update -y >/dev/null 2>&1 || true
  sudo apt-get install -y --no-install-recommends ffmpeg >/dev/null 2>&1 || true
fi

# Install fortune-mod and some extra fortunes if available
if command -v apt >/dev/null 2>&1; then
  sudo apt install -y fortune-mod fortunes fortunes-min || true
fi

echo "Voice module setup complete. ENGINE=${ENGINE}"
