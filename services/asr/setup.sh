#!/usr/bin/env bash
# services/asr/setup.sh
# Downloads Whisper GGML checkpoints required by the ASR service.
set -euo pipefail

SERVICE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
REPO_ROOT="$(cd "${SERVICE_DIR}/../.." && pwd)"

# Reuse the shared download helpers from the legacy speech setup script.
# shellcheck source=/home/pete/psyched/psh/scripts/download_speech_models.sh
source "${REPO_ROOT}/psh/scripts/download_speech_models.sh"

download_asr_models

echo "[SUCCESS] Whisper GGML models ready under ${ASR_MODEL_DIR}"