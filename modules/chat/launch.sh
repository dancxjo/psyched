#!/usr/bin/env bash
set -euo pipefail

"${BASH_SOURCE[0]}" &>/dev/null || true

# Launch the chat node. Requires that the workspace is built and env is sourced.

# Resolve host config file
HOSTNAME_short="${HOST:-$(hostname -s)}"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CFG_FILE="${REPO_DIR}/hosts/${HOSTNAME_short}/config/chat.toml"

cfg_get() {
  local key="$1"; local def_val="${2:-}"
  if [ -f "$CFG_FILE" ]; then
    local line
    line="$(grep -E "^${key}=" "$CFG_FILE" | tail -n1 || true)"
    if [ -n "$line" ]; then echo "${line#*=}"; return 0; fi
  fi
  echo "$def_val"
}

SYSTEM_PROMPT_VAL="$(cfg_get system_prompt "${CHAT_SYSTEM_PROMPT:-You_are_a_helpful_assistant._Reply_in_one_concise_sentence.}")"
CONV_TOPIC_VAL="$(cfg_get conversation_topic "${CHAT_CONVERSATION_TOPIC:-/conversation}")"
VOICE_TOPIC_VAL="$(cfg_get voice_topic "${CHAT_VOICE_TOPIC:-/voice}")"
MODEL_VAL="$(cfg_get model "${CHAT_MODEL:-gemma3}")"
OLLAMA_HOST_VAL="$(cfg_get ollama_host "${OLLAMA_HOST:-http://localhost:11434}")"
MAX_HISTORY_VAL="$(cfg_get max_history "20")"

ros2 launch chat chat.launch.py \
  system_prompt:="${SYSTEM_PROMPT_VAL}" \
  conversation_topic:="${CONV_TOPIC_VAL}" \
  voice_topic:="${VOICE_TOPIC_VAL}" \
  model:="${MODEL_VAL}" \
  ollama_host:="${OLLAMA_HOST_VAL}" \
  max_history:="${MAX_HISTORY_VAL}" \
  ${@:-}
