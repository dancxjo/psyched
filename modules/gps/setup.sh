#!/usr/bin/env bash
set -euo pipefail

# Config: source ../../config/gps.env from real script location
REAL_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$REAL_PATH")"
MODULE_NAME="$(basename "$(dirname "$REAL_PATH")")"
CONF_FILE="$(cd "$SCRIPT_DIR/../.." && pwd)/config/${MODULE_NAME}.env"
if [ -f "$CONF_FILE" ]; then
  # shellcheck disable=SC1090
  . "$CONF_FILE"
fi

# Determine repository root regardless of current working directory
if REPO_DIR_GIT_ROOT=$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel 2>/dev/null); then
  REPO_DIR="$REPO_DIR_GIT_ROOT"
else
  REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
fi
SRC_DIR="${REPO_DIR}/src"
PKG_DIR_MODULE="${REPO_DIR}/modules/${MODULE_NAME}/packages"

mkdir -p "${SRC_DIR}" "${PKG_DIR_MODULE}"

# Link required packages into src/ from module-local packages only. If missing,
# print warnings so the user can fix the module package layout.
if [ -d "${PKG_DIR_MODULE}/ublox_gps" ]; then
  ln -sfn "${PKG_DIR_MODULE}/ublox_gps" "${SRC_DIR}/ublox_gps"
else
  echo "[gps/setup] Warning: module-local package not found: ${PKG_DIR_MODULE}/ublox_gps" >&2
fi

if [ -d "${PKG_DIR_MODULE}/psyched_msgs" ]; then
  ln -sfn "${PKG_DIR_MODULE}/psyched_msgs" "${SRC_DIR}/psyched_msgs"
else
  echo "[gps/setup] Warning: module-local package not found: ${PKG_DIR_MODULE}/psyched_msgs" >&2
fi

# System setup for u-blox 7 using gpsd
# - Install gpsd and clients
# - Create udev rule to symlink /dev/gps0 for u-blox devices
# - Add current user to dialout group for serial access
# - Configure /etc/default/gpsd to use /dev/gps0 and enable service

if command -v apt >/dev/null 2>&1; then
  echo "[gps/setup] Installing gpsd and tools..."
  sudo apt update
  sudo apt install -y gpsd gpsd-clients || true
fi

# Add user to dialout for serial device permissions
if id -nG "$USER" | grep -qw dialout; then
  echo "[gps/setup] User $USER already in dialout group"
else
  echo "[gps/setup] Adding $USER to dialout group (log out/in required)"
  sudo usermod -aG dialout "$USER" || true
fi

# Create udev rule to create /dev/gps0 symlink for u-blox 7
UDEV_RULE_FILE="/etc/udev/rules.d/99-gps-ublox.rules"
UDEV_RULE_CONTENT='SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a7", SYMLINK+="gps0"'
if [ "${GPS_SKIP_UDEV:-0}" != "1" ]; then
  echo "[gps/setup] Creating udev rule for u-blox 7 at ${UDEV_RULE_FILE}"
  echo "$UDEV_RULE_CONTENT" | sudo tee "$UDEV_RULE_FILE" >/dev/null
  echo "[gps/setup] Reloading udev rules"
  sudo udevadm control --reload-rules
  sudo udevadm trigger || true
else
  echo "[gps/setup] Skipping udev rule creation per GPS_SKIP_UDEV=1"
fi

# Configure gpsd defaults
GPSD_DEFAULT="/etc/default/gpsd"
if [ -w "$GPSD_DEFAULT" ] || [ -w "$(dirname "$GPSD_DEFAULT")" ]; then
  echo "[gps/setup] Configuring $GPSD_DEFAULT"
  sudo sed -i 's/^START_DAEMON=.*/START_DAEMON="true"/' "$GPSD_DEFAULT" || true
  sudo sed -i 's#^USBAUTO=.*#USBAUTO="true"#' "$GPSD_DEFAULT" || true
  if grep -q '^DEVICES=' "$GPSD_DEFAULT"; then
    sudo sed -i 's#^DEVICES=.*#DEVICES="/dev/gps0"#' "$GPSD_DEFAULT" || true
  else
    echo 'DEVICES="/dev/gps0"' | sudo tee -a "$GPSD_DEFAULT" >/dev/null
  fi
else
  echo "[gps/setup] Warning: cannot modify $GPSD_DEFAULT (permissions). Configure manually if needed."
fi

# Enable and start gpsd service
if command -v systemctl >/dev/null 2>&1 && [ -d "/run/systemd/system" ]; then
  echo "[gps/setup] Enabling and starting gpsd service"
  if ! systemctl is-enabled gpsd.service >/dev/null 2>&1; then
    sudo systemctl enable gpsd.service || true
  fi
  sudo systemctl restart gpsd.service || sudo systemctl start gpsd.service || true
else
  echo "[gps/setup] Systemd not available; you may need to run: gpsd -N -n /dev/gps0 &"
fi

# Python dependency for client node (gps3)
if ! python3 -c 'import gps3' >/dev/null 2>&1; then
  echo "[gps/setup] Installing gps3 Python client library"
  if sudo pip3 install --break-system-packages gps3 >/dev/null 2>&1 || sudo pip3 install --break-system-packages gps3 >/dev/null 2>&1; then
    echo "[gps/setup] Installed gps3"
  else
    echo "[gps/setup] Warning: Failed to install gps3; ublox_gps node may not run until installed" >&2
  fi
else
  echo "[gps/setup] gps3 already present"
fi

echo "[gps/setup] Done. Next: make build"
