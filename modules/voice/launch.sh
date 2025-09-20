#!/usr/bin/env bash
set -euo pipefail

# Config: source hosts/<host>/config/voice.env if present relative to repo root, else fallback to ./config/voice.env
REAL_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$REAL_PATH")"          # .../modules/voice
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
HOST_SHORT="${HOST:-$(hostname -s)}"
CONF_FILE=""
if [ -f "${REPO_DIR}/hosts/${HOST_SHORT}/config/voice.env" ]; then
	CONF_FILE="${REPO_DIR}/hosts/${HOST_SHORT}/config/voice.env"
elif [ -f "${REPO_DIR}/hosts/$(hostname -s)/config/voice.env" ]; then
	CONF_FILE="${REPO_DIR}/hosts/$(hostname -s)/config/voice.env"
elif [ -f "${REPO_DIR}/config/voice.env" ]; then
	CONF_FILE="${REPO_DIR}/config/voice.env"
fi
if [ -n "$CONF_FILE" ] && [ -f "$CONF_FILE" ]; then
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

# Prefer configured PIPER_VOICES_DIR, else fall back to repo voices dir
VOICES_DIR_VAL="${PIPER_VOICES_DIR:-${REPO_DIR}/voices}"

ros2 launch voice voice.launch.py \
	engine:="${ENGINE_VAL}" \
	topic:="${TOPIC_VAL}" \
	pause_topic:="${PAUSE_VAL}" \
	resume_topic:="${RESUME_VAL}" \
	clear_topic:="${CLEAR_VAL}" \
	interrupt_topic:="${INTERRUPT_VAL}" \
	model:="${MODEL_VAL}" \
	voices_dir:="${VOICES_DIR_VAL}" \
	${@:-}
