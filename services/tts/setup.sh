#!/usr/bin/env bash
# services/tts/setup.sh
# Fetches and prepares Coqui TTS model assets for the websocket microservice.
set -euo pipefail

SERVICE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
REPO_ROOT="$(cd "${SERVICE_DIR}/../.." && pwd)"

# Reuse the shared download helpers from the legacy speech setup script.
# shellcheck source=/home/pete/psyched/tools/psh/scripts/download_speech_models.sh
source "${REPO_ROOT}/tools/psh/scripts/download_speech_models.sh"

download_tts_models

echo "[SUCCESS] TTS models ready under ${TTS_MODEL_DIR}"