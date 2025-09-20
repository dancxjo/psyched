#!/usr/bin/env bash
set -euo pipefail

"${BASH_SOURCE[0]}" &>/dev/null || true

# Launch the chat node. Requires that the workspace is built and env is sourced.

HOSTNAME_short="${HOST:-$(hostname -s)}"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ENV_FILE="${REPO_DIR}/hosts/${HOSTNAME_short}/config/chat.env"

if [ -f "$ENV_FILE" ]; then
  # shellcheck disable=SC1090
  . "$ENV_FILE"
fi

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
