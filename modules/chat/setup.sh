#!/usr/bin/env bash
set -euo pipefail

# Determine script location and repo-relative paths
REAL_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$REAL_PATH")"

# Optional per-module config
CONF_FILE="${SCRIPT_DIR}/../../config/chat.env"
if [ -f "$CONF_FILE" ]; then
  # shellcheck disable=SC1090
  . "$CONF_FILE"
fi

# Chat module setup: ensure packages are linked from module-local packages only
# Determine repository root regardless of current working directory
if REPO_DIR_GIT_ROOT=$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel 2>/dev/null); then
  REPO_DIR="$REPO_DIR_GIT_ROOT"
else
  REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
fi
SRC_DIR="${REPO_DIR}/src"
PKG_DIR_MODULE="${REPO_DIR}/modules/chat/packages"

mkdir -p "${SRC_DIR}" "${PKG_DIR_MODULE}"

# Link required packages into src/ from module-local packages only. Warn if missing.
if [ -d "${PKG_DIR_MODULE}/chat" ]; then
  ln -sfn "${PKG_DIR_MODULE}/chat" "${SRC_DIR}/chat"
else
  echo "[chat/setup] Warning: module-local package not found: ${PKG_DIR_MODULE}/chat" >&2
fi

if [ -d "${PKG_DIR_MODULE}/psyched_msgs" ]; then
  ln -sfn "${PKG_DIR_MODULE}/psyched_msgs" "${SRC_DIR}/psyched_msgs"
else
  echo "[chat/setup] Warning: module-local package not found: ${PKG_DIR_MODULE}/psyched_msgs" >&2
fi

if [ -d "${PKG_DIR_MODULE}/voice" ]; then
  ln -sfn "${PKG_DIR_MODULE}/voice" "${SRC_DIR}/voice"
else
  echo "[chat/setup] Warning: module-local package not found: ${PKG_DIR_MODULE}/voice" >&2
fi

# Install Python dependencies for chat module
echo "[chat/setup] Installing Python dependencies..."
if ! python3 -c 'import requests' >/dev/null 2>&1; then
  echo "[chat/setup] Installing requests library..."
  if pip3 install --break-system-packages requests >/dev/null 2>&1 || sudo pip3 install --break-system-packages requests >/dev/null 2>&1; then
    echo "[chat/setup] Installed requests"
  else
    echo "[chat/setup] Warning: Failed to install requests; chat module may not function properly" >&2
  fi
else
  echo "[chat/setup] requests already present"
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

# Enable and start the Ollama service (systemd) or fall back to background serve
if command -v ollama >/dev/null 2>&1; then
  echo "[chat/setup] Enabling and starting Ollama service..."
  if command -v systemctl >/dev/null 2>&1 && [ -d "/run/systemd/system" ]; then
    CTL="systemctl"
    if [ "$(id -u)" -ne 0 ]; then
      if command -v sudo >/dev/null 2>&1; then
        CTL="sudo systemctl"
      else
        CTL=""
        echo "[chat/setup] Warning: need root privileges to manage systemd. Run: sudo systemctl enable --now ollama"
      fi
    fi
    if [ -n "$CTL" ]; then
      $CTL daemon-reload || true
      $CTL enable ollama >/dev/null 2>&1 || echo "[chat/setup] Warning: could not enable ollama service"
      $CTL start ollama >/dev/null 2>&1 || echo "[chat/setup] Warning: could not start ollama service"
    fi
  else
    # Non-systemd environment: start ollama serve if not already running
    if ! pgrep -f "ollama serve" >/dev/null 2>&1; then
      echo "[chat/setup] systemd not available; starting ollama in background"
      nohup ollama serve >/dev/null 2>&1 &
      disown || true
    else
      echo "[chat/setup] ollama already running"
    fi
  fi
fi

OLLAMA_MODEL="${OLLAMA_MODEL:-tinyllama}"

echo "[chat/setup] Ensuring model '$OLLAMA_MODEL' is available..."
if command -v ollama >/dev/null 2>&1; then
  set +e
  ollama list | awk '{print $1}' | grep -qx "$OLLAMA_MODEL"
  HAS_MODEL=$?
  set -e
  if [ "$HAS_MODEL" -ne 0 ]; then
    echo "[chat/setup] Pulling $OLLAMA_MODEL... (this may take a while)"
    ollama pull "$OLLAMA_MODEL" || echo "[chat/setup] Warning: could not pull $OLLAMA_MODEL now. Ensure it exists before running chat."
  fi
fi

echo "[chat/setup] Done. Build with: make build"
