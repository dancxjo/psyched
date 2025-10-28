#!/usr/bin/env bash
set -euo pipefail

# modules/conversant/scripts/ensure_ollama_model.sh
# Makes sure the requested Ollama model is available on the local host.

log() {
  echo "[conversant][ollama-model] $*" >&2
}

run_as_root() {
  if [[ "$(id -u)" -eq 0 ]]; then
    "$@"
  else
    if command -v sudo >/dev/null 2>&1; then
      sudo "$@"
    else
      log "sudo unavailable; rerun as root to manage Ollama service."
      exit 1
    fi
  fi
}

trim_trailing_slash() {
  local value=$1
  while [[ "${value}" == */ ]]; do
    value=${value%/}
  done
  printf '%s' "${value}"
}

derive_host() {
  local configured=${CONVERSANT_LOCAL_LLM_URL:-${OLLAMA_HOST:-}}
  if [[ -n "${configured}" ]]; then
    configured=${configured%%/api/*}
    configured=$(trim_trailing_slash "${configured}")
  fi

  if [[ -z "${configured}" ]]; then
    configured="http://127.0.0.1:11434"
  fi

  printf '%s' "${configured}"
}

start_daemon_if_possible() {
  if ! command -v systemctl >/dev/null 2>&1; then
    return
  fi

  if ! systemctl list-unit-files --type=service | grep -q '^ollama\.service'; then
    return
  fi

  if run_as_root systemctl is-active --quiet ollama >/dev/null 2>&1; then
    return
  fi

  if run_as_root systemctl start ollama >/dev/null 2>&1; then
    log "Started ollama.service to prepare the model."
  else
    log "Warning: unable to start ollama.service automatically."
  fi
}

wait_for_daemon() {
  local attempts=${1:-10}
  local delay=${2:-2}
  local attempt=1
  while (( attempt <= attempts )); do
    if ollama list >/dev/null 2>&1; then
      return 0
    fi
    sleep "${delay}"
    attempt=$((attempt + 1))
  done
  return 1
}

ensure_model() {
  local model=${CONVERSANT_LOCAL_LLM_MODEL:-${OLLAMA_MODEL:-gemma3:latest}}
  local host
  host=$(derive_host)
  export OLLAMA_HOST="${host}"

  if ! command -v ollama >/dev/null 2>&1; then
    log "Ollama CLI not found; run install_ollama.sh first."
    exit 1
  fi

  start_daemon_if_possible

  if ! wait_for_daemon 10 3; then
    log "Ollama daemon did not become ready at ${OLLAMA_HOST}."
    exit 1
  fi

  if ollama show "${model}" >/dev/null 2>&1; then
    log "Model ${model} already available."
    return
  fi

  log "Pulling Ollama model ${model} using host ${OLLAMA_HOST}."
  ollama pull "${model}"
  log "Model ${model} fetched successfully."
}

ensure_model

