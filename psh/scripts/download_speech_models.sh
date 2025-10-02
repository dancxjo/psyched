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
ASR_MODEL_DIR="${REPO_ROOT}/asr-service/models"

# Create directories if they do not exist
mkdir -p "$LLM_MODEL_DIR" "$TTS_MODEL_DIR" "$ASR_MODEL_DIR"

# Support Hugging Face token via environment variable.
# It will look for, in order: HUGGINGFACE_HUB_TOKEN, HUGGINGFACE_TOKEN, HF_HUB_TOKEN, HF_TOKEN
HF_TOKEN="${HUGGINGFACE_HUB_TOKEN:-${HUGGINGFACE_TOKEN:-${HF_HUB_TOKEN:-${HF_TOKEN:-}}}}"
if [ -n "$HF_TOKEN" ]; then
  # Bash array so we can expand into curl args conditionally
  AUTH_HEADER=( -H "Authorization: Bearer $HF_TOKEN" )
else
  AUTH_HEADER=()
fi

# Small helper that wraps curl so we can capture HTTP status codes and
# produce better diagnostics (especially for 401 Unauthorized responses).
download() {
  local url="$1"
  local out="$2"
  local retries="${3:-5}"
  local retry_delay="${4:-5}"

  mkdir -p "$(dirname "$out")"

  # Use a temp file so partial downloads don't clobber an existing file.
  local tmp="${out}.tmp"

  # Run curl and capture HTTP status in a separate variable.
  # -S shows error, -s hides progress, -L follows redirects, -w writes status code
  local http_code
  if ! http_code=$(curl -sS -w "%{http_code}" -L --retry "$retries" --retry-delay "$retry_delay" \
      --continue-at - "${AUTH_HEADER[@]}" -o "$tmp" "$url"); then
    echo "[ERROR] curl failed while fetching $url"
    rm -f "$tmp" || true
    return 22
  fi

  # Move tmp to final file only if we got a 2xx status code
  if [[ "$http_code" =~ ^2[0-9]{2}$ ]]; then
    mv "$tmp" "$out"
    return 0
  fi

  # Helpful diagnostics for common failure modes
  echo "[ERROR] Failed to download $url (HTTP $http_code)"
  if [ "$http_code" = "401" ] || [ "$http_code" = "403" ]; then
    echo "[ERROR] The Hugging Face URL returned HTTP $http_code, which usually means the model requires authentication."
    echo "[HINT] Export a Hugging Face token into one of these env vars and re-run:" \
         "HUGGINGFACE_HUB_TOKEN, HUGGINGFACE_TOKEN, HF_HUB_TOKEN, or HF_TOKEN"
    echo "  export HUGGINGFACE_HUB_TOKEN=\"<your-token>\""
    echo "See https://huggingface.co/settings/tokens to create a token."
    echo "If you prefer, log in locally with 'huggingface-cli login' and re-run this script."
  else
    # Dump a short snippet of the response body to help debugging (HTML error pages, LFS pointers)
    echo "[INFO] Response body (first 400 bytes) for debugging:" || true
    head -c 400 "$tmp" 2>/dev/null | sed -n '1,200p' || true
  fi

  rm -f "$tmp" || true
  return 22
}

echo "[INFO] Downloading LLM model (GGUF) for forebrain-llm..."
LLM_MODEL_NAME="gpt-oss-20b-Q5_K_M.gguf"
LLM_MODEL_PATH="$LLM_MODEL_DIR/$LLM_MODEL_NAME"
if [ ! -f "$LLM_MODEL_PATH" ]; then
  # Prefer the canonical resolve URL without the extra query param which can
  # sometimes produce HTML wrappers or redirects depending on HF settings.
  HF_MODEL_URL="https://huggingface.co/unsloth/gpt-oss-20b-GGUF/resolve/main/$LLM_MODEL_NAME"
  echo "[INFO] Fetching $LLM_MODEL_NAME from $HF_MODEL_URL"
  if ! download "$HF_MODEL_URL" "$LLM_MODEL_PATH" 5 5; then
    echo "[ERROR] Failed to download LLM model. See messages above for details and check your Hugging Face token/permissions."
    exit 22
  fi

  # Quick sanity-check: GGUF files start with ASCII 'GGUF' (bytes 47 47 55 46).
  # Some common failure modes are: an HTML error page, a Git LFS pointer file,
  # or an auth/login HTML response. Detect those and print a clear diagnostic.
  check_magic() {
    # Read first 4 bytes as hex (upper-case, no separator)
    local magic
    magic=$(hexdump -n4 -v -e '4/1 "%02X"' "$1" 2>/dev/null || true)
    echo "$magic"
  }

  MAGIC_HEX=$(check_magic "$LLM_MODEL_PATH")
  if [ -z "$MAGIC_HEX" ]; then
    echo "[ERROR] Could not read header from $LLM_MODEL_PATH"
    file "$LLM_MODEL_PATH" || true
    exit 23
  fi

  # Expected: 47 47 55 46 -> 47475546. Some tools/platforms may show the bytes
  # in little-endian order; treat the reversed sequence as a warning as well.
  if [ "$MAGIC_HEX" != "47475546" ] && [ "$MAGIC_HEX" != "46554747" ]; then
    echo "[ERROR] Downloaded file does not look like a GGUF model (magic=0x$MAGIC_HEX)"
    echo "[ERROR] Common causes: you downloaded an HTML error page, a Git LFS pointer, or the file is incomplete."
    echo "[ERROR] File type:"; file "$LLM_MODEL_PATH" || true
    echo "[ERROR] First 200 bytes (for debugging):"; head -c 200 "$LLM_MODEL_PATH" | sed -n '1,200p' || true
    echo "[HINT] If the file contains 'git-lfs' or 'version https://git-lfs.github.com', you need to pull LFS objects or download via the Hugging Face web UI with credentials."
    echo "[HINT] If the file looks like HTML (starts with '<!doctype' or '<html'), export HUGGINGFACE_HUB_TOKEN and re-run this script, or use 'huggingface-cli login' locally."
    echo "See https://huggingface.co/settings/tokens to create a token."
    exit 24
  fi
else
  echo "[INFO] LLM model already present."
fi

echo "[INFO] Downloading TTS model for tts-websocket..."
# Example: Download en_US-amy-medium.onnx (adjust as needed)
if [ ! -f "$TTS_MODEL_DIR/en_US-amy-medium.onnx" ]; then
  if ! download "https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/amy/medium/en_US-amy-medium.onnx" "$TTS_MODEL_DIR/en_US-amy-medium.onnx"; then
    echo "[ERROR] Failed to download TTS model. See messages above for details."
    exit 22
  fi
else
  echo "[INFO] TTS model already present."
fi

echo "[INFO] Downloading ASR models for asr-service..."
# The ggerganov/whisper.cpp repo provides ggml-converted Whisper models
# (ggml-*.bin). We no longer attempt to download CT2 zip packages (tiny.en-ct2)
# because those files are not present in that repo. The script downloads the
# ggml binaries below (ggml-tiny.en.bin, ggml-small.en.bin, etc.).

GGML_MODEL_PATH="$ASR_MODEL_DIR/ggml-tiny.en.bin"
if [ ! -f "$GGML_MODEL_PATH" ]; then
  if ! download "https://huggingface.co/ggerganov/whisper.cpp/resolve/main/ggml-tiny.en.bin" "$GGML_MODEL_PATH"; then
    echo "[ERROR] Failed to download ggml ASR model. See messages above for details."
    exit 22
  fi
else
  echo "[INFO] ggml ASR model already present."
fi

SMALL_MODEL_PATH="$ASR_MODEL_DIR/ggml-small.en.bin"
if [ ! -f "$SMALL_MODEL_PATH" ]; then
  if ! download "https://huggingface.co/ggerganov/whisper.cpp/resolve/main/ggml-small.en.bin" "$SMALL_MODEL_PATH"; then
    echo "[ERROR] Failed to download ggml small ASR model. See messages above for details."
    exit 22
  fi
else
  echo "[INFO] ggml small ASR model already present."
fi

LARGE_MODEL_PATH="$ASR_MODEL_DIR/ggml-large-v3-q5_0.bin"
if [ ! -f "$LARGE_MODEL_PATH" ]; then
  if ! download "https://huggingface.co/ggerganov/whisper.cpp/resolve/main/ggml-large-v3-q5_0.bin" "$LARGE_MODEL_PATH"; then
    echo "[ERROR] Failed to download ggml large ASR model. See messages above for details."
    exit 22
  fi
else
  echo "[INFO] ggml large ASR model already present."
fi

echo "[SUCCESS] All models downloaded and folders set up."
