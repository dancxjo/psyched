#!/usr/bin/env bash
# psh/scripts/download_speech_models.sh
# Downloads required models for ASR, TTS, and LLM microservices for compose/speech-stack.compose.yml
# Usage: ./psh/scripts/download_speech_models.sh
set -euo pipefail

# Directories
LLM_MODEL_DIR="$(dirname "$0")/../../forebrain-llm/models"
TTS_MODEL_DIR="$(dirname "$0")/../../tts-websocket/models"
ASR_MODEL_DIR="$(dirname "$0")/../../asr-fast/models"

# Create directories if they do not exist
mkdir -p "$LLM_MODEL_DIR" "$TTS_MODEL_DIR" "$ASR_MODEL_DIR"

echo "[INFO] Downloading LLM model (GGUF) for forebrain-llm..."
# Example: Download llama3-8b-instruct.Q4_K_M.gguf (adjust as needed)
if [ ! -f "$LLM_MODEL_DIR/llama3-8b-instruct.Q4_K_M.gguf" ]; then
  curl -L -o "$LLM_MODEL_DIR/llama3-8b-instruct.Q4_K_M.gguf" \
    "https://huggingface.co/TheBloke/Llama-3-8B-Instruct-GGUF/resolve/main/llama-3-8b-instruct.Q4_K_M.gguf"
else
  echo "[INFO] LLM model already present."
fi

echo "[INFO] Downloading TTS model for tts-websocket..."
# Example: Download en_US-amy-medium.onnx (adjust as needed)
if [ ! -f "$TTS_MODEL_DIR/en_US-amy-medium.onnx" ]; then
  curl -L -o "$TTS_MODEL_DIR/en_US-amy-medium.onnx" \
    "https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/amy/medium/en_US-amy-medium.onnx"
else
  echo "[INFO] TTS model already present."
fi

echo "[INFO] Downloading ASR model for asr-fast..."
# Example: Download tiny.en-ct2 (adjust as needed)
if [ ! -f "$ASR_MODEL_DIR/tiny.en-ct2.zip" ]; then
  curl -L -o "$ASR_MODEL_DIR/tiny.en-ct2.zip" \
    "https://huggingface.co/openai/whisper/resolve/main/tiny.en-ct2.zip"
  unzip -o "$ASR_MODEL_DIR/tiny.en-ct2.zip" -d "$ASR_MODEL_DIR"
else
  echo "[INFO] ASR model already present."
fi

echo "[SUCCESS] All models downloaded and folders set up."
