#!/bin/bash
set -euo pipefail
REPO_DIR="$(pwd)"
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
  source "$CONF_FILE"
fi
ENGINE_VAL="${VOICE_ENGINE:-espeak}"
TOPIC_VAL="${VOICE_TOPIC:-/voice}"
PAUSE_VAL="${VOICE_PAUSE:-${VOICE_INTERRUPT:-/voice/interrupt}}"
RESUME_VAL="${VOICE_RESUME:-/voice/resume}"
CLEAR_VAL="${VOICE_CLEAR:-/voice/clear}"
INTERRUPT_VAL="${VOICE_INTERRUPT:-/voice/interrupt}"
MODEL_VAL="${VOICE_MODEL:-en_US-ryan-high}"
VOICES_DIR_VAL="${PIPER_VOICES_DIR:-${REPO_DIR}/voices}"
exec ros2 launch voice voice.launch.py \
  engine:="${ENGINE_VAL}" \
  topic:="${TOPIC_VAL}" \
  pause_topic:="${PAUSE_VAL}" \
  resume_topic:="${RESUME_VAL}" \
  clear_topic:="${CLEAR_VAL}" \
  interrupt_topic:="${INTERRUPT_VAL}" \
  model:="${MODEL_VAL}" \
  voices_dir:="${VOICES_DIR_VAL}"
