#!/usr/bin/env bash
set -euo pipefail

# Chat module setup: ensure packages are linked and Ollama with gemma3 is installed

REPO_DIR="$(pwd)"
SRC_DIR="${REPO_DIR}/src"
PKG_DIR="${REPO_DIR}/packages"

mkdir -p "${SRC_DIR}" "${PKG_DIR}"

# Clean current src and link chat, voice, and msgs packages
find "${SRC_DIR}" -mindepth 1 -maxdepth 1 -exec rm -rf {} +
ln -sfn "${PKG_DIR}/chat" "${SRC_DIR}/chat"
ln -sfn "${PKG_DIR}/psyched_msgs" "${SRC_DIR}/psyched_msgs"
if [ -d "${PKG_DIR}/voice" ]; then
  ln -sfn "${PKG_DIR}/voice" "${SRC_DIR}/voice"
fi

echo "[chat/setup] Ensuring Ollama is installed..."
if ! command -v ollama >/dev/null 2>&1; then
  echo "[chat/setup] Installing Ollama..."
  if command -v curl >/dev/null 2>&1; then
    curl -fsSL https://ollama.com/install.sh | sh
  else
    echo "[chat/setup] curl not available; please install ollama manually: https://ollama.com/"
  fi
fi

echo "[chat/setup] Ensuring model 'gemma3' is available..."
if command -v ollama >/dev/null 2>&1; then
  set +e
  ollama list | grep -q "^gemma3\b"
  HAS_GEMMA=$?
  set -e
  if [ "$HAS_GEMMA" -ne 0 ]; then
    echo "[chat/setup] Pulling gemma3... (this may take a while)"
    ollama pull gemma3 || echo "[chat/setup] Warning: could not pull gemma3 now. Ensure it exists before running chat."
  fi
fi

echo "[chat/setup] Done. Build with: make build"
