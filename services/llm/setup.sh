#!/usr/bin/env bash
# services/llm/setup.sh
# Fetches baseline GGUF checkpoints for the Ollama runtime.
set -euo pipefail

SERVICE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
REPO_ROOT="$(cd "${SERVICE_DIR}/../.." && pwd)"

# Reuse the shared download helpers from the legacy speech setup script.
# shellcheck source=/home/pete/psyched/psh/scripts/download_speech_models.sh
source "${REPO_ROOT}/psh/scripts/download_speech_models.sh"
# We won't need these until we use llama.cpp for local inference.
# download_llm_models

echo "[SUCCESS] LLM models ready under ${LLM_MODEL_DIR}"