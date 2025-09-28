#!/usr/bin/env bash
# psh/scripts/download_speech_models.sh
# Downloads required models for ASR, TTS, and LLM microservices for compose/speech-stack.compose.yml
# Usage: ./psh/scripts/download_speech_models.sh
set -euo pipefail

# Resolve repository root relative to this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Directories
LLM_MODEL_DIR="${REPO_ROOT}/forebrain-llm/models"
TTS_MODEL_DIR="${REPO_ROOT}/tools/tts_websocket/models"
ASR_MODEL_DIR="${REPO_ROOT}/asr-fast/models"

# Create directories if they do not exist
mkdir -p "$LLM_MODEL_DIR" "$TTS_MODEL_DIR" "$ASR_MODEL_DIR"

# Support Hugging Face token via environment variable.
# It will look for, in order: HUGGINGFACE_HUB_TOKEN, HUGGINGFACE_TOKEN, HF_TOKEN
HF_TOKEN="${HUGGINGFACE_HUB_TOKEN:-${HUGGINGFACE_TOKEN:-${HF_TOKEN:-}}}"
if [ -n "$HF_TOKEN" ]; then
  # Bash array so we can expand into curl args conditionally
  AUTH_HEADER=( -H "Authorization: Bearer $HF_TOKEN" )
else
  AUTH_HEADER=()
fi

echo "[INFO] Downloading LLM model (GGUF) for forebrain-llm..."
# Example: Download llama3-8b-instruct.Q4_K_M.gguf (adjust as needed)
if [ ! -f "$LLM_MODEL_DIR/llama3-8b-instruct.Q4_K_M.gguf" ]; then
  if ! curl -fSL "${AUTH_HEADER[@]}" -o "$LLM_MODEL_DIR/llama3-8b-instruct.Q4_K_M.gguf" \
    "https://huggingface.co/TheBloke/Llama-3-8B-Instruct-GGUF/resolve/main/llama-3-8b-instruct.Q4_K_M.gguf"; then
    echo "[ERROR] Failed to download LLM model. If this model is gated on Hugging Face, export a token first:"
    echo "  export HUGGINGFACE_HUB_TOKEN=\"<your-token>\""
    echo "See https://huggingface.co/settings/tokens to create a token."
    exit 22
  fi
else
  echo "[INFO] LLM model already present."
fi

echo "[INFO] Downloading TTS model for tts-websocket..."
# Example: Download en_US-amy-medium.onnx (adjust as needed)
if [ ! -f "$TTS_MODEL_DIR/en_US-amy-medium.onnx" ]; then
  if ! curl -fSL "${AUTH_HEADER[@]}" -o "$TTS_MODEL_DIR/en_US-amy-medium.onnx" \
    "https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/amy/medium/en_US-amy-medium.onnx"; then
    echo "[ERROR] Failed to download TTS model. If this file requires auth, set a Hugging Face token as described above."
    exit 22
  fi
else
  echo "[INFO] TTS model already present."
fi

echo "[INFO] Downloading ASR model for asr-fast..."
# Example: Download tiny.en-ct2 (adjust as needed)
if [ ! -f "$ASR_MODEL_DIR/tiny.en-ct2.zip" ]; then
  if ! curl -fSL "${AUTH_HEADER[@]}" -o "$ASR_MODEL_DIR/tiny.en-ct2.zip" \
    "https://huggingface.co/openai/whisper/resolve/main/tiny.en-ct2.zip"; then
    echo "[ERROR] Failed to download ASR model. If this file requires auth, set a Hugging Face token as described above."
    exit 22
  fi
  unzip -o "$ASR_MODEL_DIR/tiny.en-ct2.zip" -d "$ASR_MODEL_DIR"
else
  echo "[INFO] ASR model already present."
fi

echo "[SUCCESS] All models downloaded and folders set up."
