#!/usr/bin/env bash
set -euo pipefail

# Optionally remove the udev rule and disable gpsd
if [ "${GPS_TEARDOWN_REMOVE_UDEV:-0}" = "1" ]; then
  sudo rm -f /etc/udev/rules.d/99-gps-ublox.rules || true
  sudo udevadm control --reload-rules || true
  sudo udevadm trigger || true
fi

if command -v systemctl >/dev/null 2>&1 && [ -d "/run/systemd/system" ]; then
  sudo systemctl stop gpsd.service || true
  if [ "${GPS_TEARDOWN_DISABLE_SERVICE:-0}" = "1" ]; then
    sudo systemctl disable gpsd.service || true
  fi
fi

# Remove symlinks in src/
REAL_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$REAL_PATH")"
if REPO_DIR_GIT_ROOT=$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel 2>/dev/null); then
  REPO_DIR="$REPO_DIR_GIT_ROOT"
else
  REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
fi
SRC_DIR="${REPO_DIR}/src"
rm -f "${SRC_DIR}/ublox_gps" || true

