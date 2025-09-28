#!/bin/bash
set -euo pipefail

MODEL=${OLLAMA_MODEL:-phi4}
if command -v ollama >/dev/null 2>&1; then
  if ! ollama list | awk '{print $1}' | grep -qx "$MODEL"; then
    ollama pull "$MODEL" || true
  fi
fi
