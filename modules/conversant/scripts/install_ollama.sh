#!/usr/bin/env bash
set -euo pipefail

# modules/conversant/scripts/install_ollama.sh
# Installs the Ollama runtime locally and ensures the daemon is running.

log() {
  echo "[conversant][ollama] $*" >&2
}

run_as_root() {
  if [[ "$(id -u)" -eq 0 ]]; then
    "$@"
  else
    if command -v sudo >/dev/null 2>&1; then
      sudo "$@"
    else
      log "sudo unavailable; rerun as root to install Ollama."
      exit 1
    fi
  fi
}

download_installer() {
  local url=$1
  local destination=$2
  if command -v curl >/dev/null 2>&1; then
    curl -fsSL "${url}" -o "${destination}"
  elif command -v wget >/dev/null 2>&1; then
    wget -qO "${destination}" "${url}"
  else
    log "Neither curl nor wget is available to download Ollama installer."
    exit 1
  fi
}

ensure_installed() {
  local installer_url=${OLLAMA_INSTALL_SCRIPT_URL:-https://ollama.com/install.sh}
  if command -v ollama >/dev/null 2>&1; then
    log "Ollama already installed at $(command -v ollama)."
    return
  fi

  log "Installing Ollama from ${installer_url}."
  local installer
  installer=$(mktemp)
  trap '[[ -n "${installer:-}" ]] && rm -f "${installer}"' RETURN
  download_installer "${installer_url}" "${installer}"
  run_as_root sh "${installer}"
  trap - RETURN
  log "Ollama installation complete."
}

ensure_service_running() {
  if ! command -v systemctl >/dev/null 2>&1; then
    log "systemctl not available; ensure ollama serve is running manually if required."
    return
  fi

  if ! systemctl list-unit-files --type=service | grep -q '^ollama\.service'; then
    log "ollama.service not registered; skipping systemd enablement."
    return
  fi

  if ! run_as_root systemctl is-enabled --quiet ollama >/dev/null 2>&1; then
    if ! run_as_root systemctl enable ollama >/dev/null 2>&1; then
      log "Warning: failed to enable ollama.service; continuing."
    fi
  fi

  if ! run_as_root systemctl is-active --quiet ollama >/dev/null 2>&1; then
    if run_as_root systemctl restart ollama >/dev/null 2>&1; then
      log "Started ollama.service via systemd."
    elif run_as_root systemctl start ollama >/dev/null 2>&1; then
      log "Started ollama.service via systemd."
    else
      log "Warning: failed to start ollama.service; ensure the daemon is running."
    fi
  else
    log "ollama.service already active."
  fi
}

ensure_installed
ensure_service_running
