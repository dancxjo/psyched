#!/usr/bin/env bash
set -euo pipefail

# Config: source ../../config/voice.env from real script location
REAL_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$REAL_PATH")"
MODULE_NAME="$(basename "$(dirname "$REAL_PATH")")"
CONF_FILE="$(cd "$SCRIPT_DIR/../.." && pwd)/config/${MODULE_NAME}.env"
if [ -f "$CONF_FILE" ]; then
	# shellcheck disable=SC1090
	. "$CONF_FILE"
fi

ENGINE_VAL="${VOICE_ENGINE:-espeak}"
TOPIC_VAL="${VOICE_TOPIC:-/voice}"
PAUSE_VAL="${VOICE_PAUSE:-${VOICE_INTERRUPT:-/voice/interrupt}}"
RESUME_VAL="${VOICE_RESUME:-/voice/resume}"
CLEAR_VAL="${VOICE_CLEAR:-/voice/clear}"
INTERRUPT_VAL="${VOICE_INTERRUPT:-/voice/interrupt}"
MODEL_VAL="${VOICE_MODEL:-en_US-ryan-high}"

ros2 launch voice voice.launch.py \
	engine:="${ENGINE_VAL}" \
	topic:="${TOPIC_VAL}" \
	pause:="${PAUSE_VAL}" \
	resume:="${RESUME_VAL}" \
	clear:="${CLEAR_VAL}" \
	interrupt:="${INTERRUPT_VAL}" \
	model:="${MODEL_VAL}" \
	${@:-}
