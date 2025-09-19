#!/usr/bin/env bash
set -euo pipefail

# Launch the chat node. Requires that the workspace is built and env is sourced.

SYSTEM_PROMPT_VAL="${CHAT_SYSTEM_PROMPT:-You are a helpful assistant. Reply in one concise sentence.}"
CONV_TOPIC_VAL="${CHAT_CONVERSATION_TOPIC:-/conversation}"
VOICE_TOPIC_VAL="${CHAT_VOICE_TOPIC:-/voice}"
MODEL_VAL="${CHAT_MODEL:-gemma3}"
OLLAMA_HOST_VAL="${OLLAMA_HOST:-http://localhost:11434}"

ros2 launch chat chat.launch.py \
  system_prompt:="${SYSTEM_PROMPT_VAL}" \
  conversation_topic:="${CONV_TOPIC_VAL}" \
  voice_topic:="${VOICE_TOPIC_VAL}" \
  model:="${MODEL_VAL}" \
  ollama_host:="${OLLAMA_HOST_VAL}" \
  ${@:-}
