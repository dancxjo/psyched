#!/usr/bin/env bash
set -euo pipefail

MODEL=${OLLAMA_MODEL:-gpt-oss:20b}
if command -v ollama >/dev/null 2>&1; then
  if ! ollama list | awk '{print $1}' | grep -qx "$MODEL"; then
    ollama pull "$MODEL" || true
  fi
fi
