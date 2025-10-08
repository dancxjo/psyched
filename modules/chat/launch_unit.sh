#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="${REPO_DIR:-$(cd "${SCRIPT_DIR}/../.." && pwd)}"
export REPO_DIR
HOST_SHORT="${HOST:-$(hostname -s)}"

ARGS=()
LEGACY_TOML="${REPO_DIR}/hosts/${HOST_SHORT}/config/chat.toml"
if [[ -f "${LEGACY_TOML}" ]]; then
  mapfile -t LEGACY_ARGS < <(python3 "${REPO_DIR}/tools/launch_args.py" "${LEGACY_TOML}" || true)
  if [[ ${#LEGACY_ARGS[@]} -gt 0 ]]; then
    ARGS+=("${LEGACY_ARGS[@]}")
  fi
fi

DEFAULT_HOST_TOML="${REPO_DIR}/hosts/${HOST_SHORT}.toml"
if [[ -f "${DEFAULT_HOST_TOML}" ]]; then
  mapfile -t HOST_ARGS < <(python3 "${REPO_DIR}/tools/launch_args.py" --module chat "${DEFAULT_HOST_TOML}" || true)
  if [[ ${#HOST_ARGS[@]} -gt 0 ]]; then
    ARGS+=("${HOST_ARGS[@]}")
  fi
fi

if [[ -n "${CHAT_SYSTEM_PROMPT:-}" ]]; then
  ARGS+=("system_prompt:=${CHAT_SYSTEM_PROMPT}")
fi
if [[ -n "${CHAT_CONVERSATION_TOPIC:-}" ]]; then
  ARGS+=("conversation_topic:=${CHAT_CONVERSATION_TOPIC}")
fi
if [[ -n "${CHAT_VOICE_TOPIC:-}" ]]; then
  ARGS+=("voice_topic:=${CHAT_VOICE_TOPIC}")
fi
if [[ -n "${CHAT_TRANSCRIPT_TOPIC:-}" ]]; then
  ARGS+=("transcript_topic:=${CHAT_TRANSCRIPT_TOPIC}")
fi
if [[ -n "${CHAT_MODEL:-}" ]]; then
  ARGS+=("model:=${CHAT_MODEL}")
fi
if [[ -n "${CHAT_OLLAMA_HOST:-}" ]]; then
  ARGS+=("ollama_host:=${CHAT_OLLAMA_HOST}")
fi
if [[ -n "${CHAT_MAX_HISTORY:-}" ]]; then
  ARGS+=("max_history:=${CHAT_MAX_HISTORY}")
fi

exec ros2 launch chat chat.launch.py "${ARGS[@]}"
