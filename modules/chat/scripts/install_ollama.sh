#!/bin/bash
set -euo pipefail

if ! command -v ollama >/dev/null 2>&1; then
  if command -v curl >/dev/null 2>&1; then
    curl -fsSL https://ollama.com/install.sh | sh
  else
    echo "curl not found; please install Ollama manually" >&2
  fi
fi
