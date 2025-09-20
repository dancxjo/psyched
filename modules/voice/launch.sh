#!/usr/bin/env bash
set -euo pipefail

HOSTNAME_short="${HOST:-$(hostname -s)}"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ENV_FILE="${REPO_DIR}/hosts/${HOSTNAME_short}/config/voice.env"

if [ -f "$ENV_FILE" ]; then
	# shellcheck disable=SC1090
	. "$ENV_FILE"
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
