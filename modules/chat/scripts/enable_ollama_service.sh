#!/bin/bash
set -euo pipefail

if command -v ollama >/dev/null 2>&1 && command -v systemctl >/dev/null 2>&1; then
  sudo systemctl enable --now ollama || true
fi
