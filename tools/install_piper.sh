#!/usr/bin/env bash
set -euo pipefail

# Helper to install Piper voice model artifacts into a user-local voices dir
# - Downloads ONNX model and its JSON config
# - Works idempotently and prints clear next steps when network is blocked

REAL_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$REAL_PATH")"

PIPER_VOICES_DIR="${PIPER_VOICES_DIR:-$HOME/.local/piper/voices}"
VOICE_MODEL_BASENAME="${VOICE_MODEL_BASENAME:-en_US-john-medium}"
VOICE_MODEL_PATH="${VOICE_MODEL_PATH:-${PIPER_VOICES_DIR}/${VOICE_MODEL_BASENAME}.onnx}"
VOICE_CFG_PATH="${VOICE_CFG_PATH:-${PIPER_VOICES_DIR}/${VOICE_MODEL_BASENAME}.onnx.json}"
VOICE_URL="${VOICE_URL:-https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/john/medium/en_US-john-medium.onnx?download=true}"
VOICE_CFG_URL="${VOICE_CFG_URL:-https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/john/medium/en_US-john-medium.onnx.json?download=true}"

mkdir -p "${PIPER_VOICES_DIR}"

download_file() {
  local url="$1" path="$2"
  if command -v curl >/dev/null 2>&1; then
    curl -fSL -o "$path" "$url"
  elif command -v wget >/dev/null 2>&1; then
    wget -O "$path" "$url"
  else
    return 2
  fi
}

echo "[install_piper] Installing Piper voice assets into: ${PIPER_VOICES_DIR}"

if [ -f "${VOICE_MODEL_PATH}" ]; then
  echo "[install_piper] Model already exists: ${VOICE_MODEL_PATH}"
else
  echo "[install_piper] Downloading model: ${VOICE_MODEL_BASENAME}"
  set +e
  if download_file "${VOICE_URL}" "${VOICE_MODEL_PATH}"; then
    echo "[install_piper] Model downloaded -> ${VOICE_MODEL_PATH}"
  else
    rc=$?
    echo "[install_piper] Warning: failed to download model (rc=${rc})." >&2
    echo "[install_piper] If you are on an air-gapped system, place the model ONNX file at: ${VOICE_MODEL_PATH}" >&2
  fi
  set -e
fi

if [ -f "${VOICE_CFG_PATH}" ]; then
  echo "[install_piper] Config already exists: ${VOICE_CFG_PATH}"
else
  echo "[install_piper] Downloading config: ${VOICE_MODEL_BASENAME}.onnx.json"
  set +e
  if download_file "${VOICE_CFG_URL}" "${VOICE_CFG_PATH}"; then
    echo "[install_piper] Config downloaded -> ${VOICE_CFG_PATH}"
  else
    rc=$?
    echo "[install_piper] Warning: failed to download config (rc=${rc})." >&2
    echo "[install_piper] If you are on an air-gapped system, place the config JSON at: ${VOICE_CFG_PATH}" >&2
  fi
  set -e
fi

echo "[install_piper] Done. You can set PIPER_VOICE to the ONNX path:"
echo "  export PIPER_VOICE=\"${VOICE_MODEL_PATH}\""

exit 0
