#!/usr/bin/env bash
set -euo pipefail

# Config: source ../../config/<module>.env from real script location
REAL_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$REAL_PATH")"
MODULE_NAME="$(basename "$(dirname "$REAL_PATH")")"
CONF_FILE="$(cd "$SCRIPT_DIR/../.." && pwd)/config/${MODULE_NAME}.env"
if [ -f "$CONF_FILE" ]; then
  # shellcheck disable=SC1090
  . "$CONF_FILE"
fi

"${BASH_SOURCE[0]}" &>/dev/null || true

# Launch the chat node. Requires that the workspace is built and env is sourced.

SYSTEM_PROMPT_VAL="${CHAT_SYSTEM_PROMPT:-${SYSTEM_PROMPT:-You_are_a_helpful_assistant._Reply_in_one_concise_sentence.}}"
CONV_TOPIC_VAL="${CHAT_CONVERSATION_TOPIC:-${CONVERSATION_TOPIC:-/conversation}}"
VOICE_TOPIC_VAL="${CHAT_VOICE_TOPIC:-${VOICE_TOPIC:-/voice}}"
MODEL_VAL="${CHAT_MODEL:-${OLLAMA_MODEL:-tinyllama}}"
OLLAMA_HOST_VAL="${OLLAMA_HOST:-http://localhost:11434}"
MAX_HISTORY_VAL="${CHAT_MAX_HISTORY:-${MAX_HISTORY:-20}}"

ros2 launch chat chat.launch.py \
  system_prompt:="${SYSTEM_PROMPT_VAL}" \
  conversation_topic:="${CONV_TOPIC_VAL}" \
  voice_topic:="${VOICE_TOPIC_VAL}" \
  model:="${MODEL_VAL}" \
  ollama_host:="${OLLAMA_HOST_VAL}" \
  max_history:="${MAX_HISTORY_VAL}" \
  ${@:-}
