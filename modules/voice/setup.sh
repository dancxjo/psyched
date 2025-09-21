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
ENGINE="${VOICE_ENGINE:-espeak}" # espeak | piper

if [[ "${ENGINE}" == "piper" ]]; then
  # Save Piper voices in user-local directory
  PIPER_VOICES_DIR="${PIPER_VOICES_DIR:-$HOME/.local/piper/voices}"
  mkdir -p "$PIPER_VOICES_DIR" || true
  # Prefer using the centralized installer helper which handles network failures
  INSTALLER="${REPO_DIR}/tools/install_piper.sh"
  VOICE_MODEL_BASENAME="${VOICE_MODEL:-en_US-john-medium}"
  export VOICE_MODEL_BASENAME
  if [ -x "${INSTALLER}" ]; then
    echo "[voice/setup] Running Piper installer helper: ${INSTALLER}"
    if ! "${INSTALLER}"; then
      echo "[voice/setup] Warning: Piper installer returned non-zero. You may need to manually place model files in ${PIPER_VOICES_DIR}" >&2
    fi
  elif [ -f "${INSTALLER}" ]; then
    echo "[voice/setup] Installer helper found but not executable; running with bash: ${INSTALLER}"
    if ! bash "${INSTALLER}"; then
      echo "[voice/setup] Warning: Piper installer returned non-zero. You may need to manually place model files in ${PIPER_VOICES_DIR}" >&2
    fi
  else
    echo "[voice/setup] Installer helper not found: ${INSTALLER}. Falling back to inline download logic." >&2
    # Inline fallback (best-effort)
    VOICE_DIR="${PIPER_VOICES_DIR}"
    VOICE_MODEL="${VOICE_DIR}/${VOICE_MODEL_BASENAME}.onnx"
    VOICE_CFG="${VOICE_DIR}/${VOICE_MODEL_BASENAME}.onnx.json"
    VOICE_URL="${PIPER_MODEL_URL:-https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/john/medium/en_US-john-medium.onnx?download=true}"
    VOICE_CFG_URL="${PIPER_CONFIG_URL:-https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/john/medium/en_US-john-medium.onnx.json?download=true}"

    mkdir -p "${VOICE_DIR}"
    if [ ! -f "${VOICE_MODEL}" ]; then
      echo "[voice/setup] Attempting to download Piper voice model..."
      if command -v curl >/dev/null 2>&1; then
        if ! curl -fsSL -o "${VOICE_MODEL}" "${VOICE_URL}"; then
          echo "[voice/setup] Warning: could not download Piper model. Place the ONNX file at: ${VOICE_MODEL}" >&2
        fi
      else
        echo "[voice/setup] curl not available; please fetch the model and place it at: ${VOICE_MODEL}" >&2
      fi
    fi
    if [ ! -f "${VOICE_CFG}" ]; then
      echo "[voice/setup] Attempting to download Piper voice config..."
      if command -v curl >/dev/null 2>&1; then
        if ! curl -fsSL -o "${VOICE_CFG}" "${VOICE_CFG_URL}"; then
          echo "[voice/setup] Warning: could not download Piper config. Place the JSON at: ${VOICE_CFG}" >&2
        fi
      else
        echo "[voice/setup] curl not available; please fetch the config and place it at: ${VOICE_CFG}" >&2
      fi
    fi
    if [ -z "${PIPER_VOICE:-}" ]; then
      if [ -f "${VOICE_MODEL}" ]; then
        export PIPER_VOICE="${VOICE_MODEL}"
        echo "PIPER_VOICE set to ${VOICE_MODEL}"
      else
        echo "[voice/setup] PIPER_VOICE not set and model not present. Set PIPER_VOICE to the ONNX file path when available." >&2
      fi
    else
      echo "PIPER_VOICE already set to ${PIPER_VOICE}"
    fi
  fi

  # Python library is optional but helpful for fallback
  if ! python3 -c 'import piper' >/dev/null 2>&1; then
    echo "Installing optional piper-tts library (for fallback)..."
    if pip3 install --break-system-packages piper-tts >/dev/null 2>&1 || sudo pip3 install --break-system-packages piper-tts >/dev/null 2>&1; then
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
