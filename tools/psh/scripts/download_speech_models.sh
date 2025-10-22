#!/usr/bin/env bash
# tools/psh/scripts/download_speech_models.sh
# Legacy aggregator for speech-related model assets.
# Prefer running the service-specific setup scripts in services/<name>/setup.sh.
set -euo pipefail

# Resolve repository root relative to this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

# Directories
LLM_MODEL_DIR="${REPO_ROOT}/services/llm/models"
TTS_MODEL_DIR="${REPO_ROOT}/services/tts/models"
ASR_MODEL_DIR="${REPO_ROOT}/services/asr/models"

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
    echo "[HINT] Export a Hugging Face token into one of these env vars and re-run: HUGGINGFACE_HUB_TOKEN, HUGGINGFACE_TOKEN, HF_HUB_TOKEN, or HF_TOKEN"
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

check_magic() {
  local file="$1"
  hexdump -n4 -v -e '4/1 "%02X"' "$file" 2>/dev/null || true
}

download_llm_models() {
  mkdir -p "$LLM_MODEL_DIR"
  local model_name="gpt-oss-20b-Q5_K_M.gguf"
  local model_path="$LLM_MODEL_DIR/$model_name"

  echo "[INFO] Downloading LLM model (GGUF) for services/llm..."
  if [ -f "$model_path" ]; then
    echo "[INFO] LLM model already present."
    return 0
  fi

  local hf_url="https://huggingface.co/unsloth/gpt-oss-20b-GGUF/resolve/main/$model_name"
  echo "[INFO] Fetching $model_name from $hf_url"
  if ! download "$hf_url" "$model_path" 5 5; then
    echo "[ERROR] Failed to download LLM model. See messages above for details and check your Hugging Face token/permissions."
    return 22
  fi

  local magic_hex
  magic_hex=$(check_magic "$model_path")
  if [ -z "$magic_hex" ]; then
    echo "[ERROR] Could not read header from $model_path"
    file "$model_path" || true
    return 23
  fi

  if [ "$magic_hex" != "47475546" ] && [ "$magic_hex" != "46554747" ]; then
    echo "[ERROR] Downloaded file does not look like a GGUF model (magic=0x$magic_hex)"
    echo "[ERROR] Common causes: you downloaded an HTML error page, a Git LFS pointer, or the file is incomplete."
    echo "[ERROR] File type:"; file "$model_path" || true
    echo "[ERROR] First 200 bytes (for debugging):"; head -c 200 "$model_path" | sed -n '1,200p' || true
    echo "[HINT] If the file contains 'git-lfs' or 'version https://git-lfs.github.com', you need to pull LFS objects or download via the Hugging Face web UI with credentials."
    echo "[HINT] If the file looks like HTML (starts with '<!doctype' or '<html'), export HUGGINGFACE_HUB_TOKEN and re-run this script, or use 'huggingface-cli login' locally."
    echo "See https://huggingface.co/settings/tokens to create a token."
    return 24
  fi

  return 0
}

download_tts_models() {
  mkdir -p "$TTS_MODEL_DIR"
  local model_path="$TTS_MODEL_DIR/en_US-amy-medium.onnx"

  echo "[INFO] Downloading TTS model for tts-websocket..."
  if [ -f "$model_path" ]; then
    echo "[INFO] TTS model already present."
    return 0
  fi

  if ! download "https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/amy/medium/en_US-amy-medium.onnx" "$model_path"; then
    echo "[ERROR] Failed to download TTS model. See messages above for details."
    return 22
  fi

  return 0
}

download_asr_models() {
  mkdir -p "$ASR_MODEL_DIR"

  echo "[INFO] Downloading ASR models for services/asr..."

  local -a models=(
    "ggml-tiny.en.bin|https://huggingface.co/ggerganov/whisper.cpp/resolve/main/ggml-tiny.en.bin"
    "ggml-base.en.bin|https://huggingface.co/ggerganov/whisper.cpp/resolve/main/ggml-base.en.bin"
    "ggml-small.en.bin|https://huggingface.co/ggerganov/whisper.cpp/resolve/main/ggml-small.en.bin"
    "ggml-large-v3-q5_0.bin|https://huggingface.co/ggerganov/whisper.cpp/resolve/main/ggml-large-v3-q5_0.bin"
  )

  local entry
  for entry in "${models[@]}"; do
    local filename="${entry%%|*}"
    local url="${entry##*|}"
    local path="$ASR_MODEL_DIR/$filename"

    if [ -f "$path" ]; then
      echo "[INFO] $filename already present."
      continue
    fi

    if ! download "$url" "$path"; then
      echo "[ERROR] Failed to download $filename. See messages above for details."
      return 22
    fi
  done

  return 0
}

legacy_main() {
  download_llm_models
  download_tts_models
  download_asr_models
  echo "[SUCCESS] All models downloaded and folders set up."
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "[WARN] psh/scripts/download_speech_models.sh is deprecated. Run services/<name>/setup.sh instead."
  if ! legacy_main "$@"; then
    exit $?
  fi
fi
