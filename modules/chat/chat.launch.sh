#!/bin/bash
set -euo pipefail
SYSTEM_PROMPT_VAL="${CHAT_SYSTEM_PROMPT:-${SYSTEM_PROMPT:-You_are_a_helpful_assistant._Reply_in_one_concise_sentence.}}"
CONV_TOPIC_VAL="${CHAT_CONVERSATION_TOPIC:-${CONVERSATION_TOPIC:-/conversation}}"
VOICE_TOPIC_VAL="${CHAT_VOICE_TOPIC:-${VOICE_TOPIC:-/voice}}"
MODEL_VAL="${CHAT_MODEL:-${OLLAMA_MODEL:-gemma3}}"
OLLAMA_HOST_VAL="${OLLAMA_HOST:-http://localhost:11434}"
MAX_HISTORY_VAL="${CHAT_MAX_HISTORY:-${MAX_HISTORY:-20}}"

REPO_DIR="${REPO_DIR:-$(cd "$(dirname "$0")/../.." && pwd)}"
export REPO_DIR
exec ros2 launch chat chat.launch.py \
  system_prompt:="${SYSTEM_PROMPT_VAL}" \
  conversation_topic:="${CONV_TOPIC_VAL}" \
  voice_topic:="${VOICE_TOPIC_VAL}" \
  model:="${MODEL_VAL}" \
  ollama_host:="${OLLAMA_HOST_VAL}" \
  max_history:="${MAX_HISTORY_VAL}"
