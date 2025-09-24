#!/usr/bin/env bash
set -euo pipefail

# Config: source ../../config/pilot.env from real script location
REAL_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$REAL_PATH")"
MODULE_NAME="$(basename "$(dirname "$REAL_PATH")")"
CONF_FILE="$(cd "$SCRIPT_DIR/../.." && pwd)/config/${MODULE_NAME}.env"
if [ -f "$CONF_FILE" ]; then
  # shellcheck disable=SC1090
  . "$CONF_FILE"
fi


# Pilot module setup: build local packages using a fresh src/ populated by symlinks.

case "${ROS_DISTRO}" in
  jade|kinetic|lunar|melodic|noetic)
    echo "Skipping end-of-life ROS distro: ${ROS_DISTRO}"
    exit 0
    ;;
  jazzy|kilted|rolling)
    echo "Detected supported ROS2 distro: ${ROS_DISTRO}"
    ;;
  *)
    echo "Proceeding with generic ROS2 distro: ${ROS_DISTRO}"
    ;;
esac
# Determine repository root regardless of current working directory
if REPO_DIR_GIT_ROOT=$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel 2>/dev/null); then
  REPO_DIR="$REPO_DIR_GIT_ROOT"
else
  REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
fi
SRC_DIR="${REPO_DIR}/src"
# Packages located under modules/<module>/packages/<module>
PKG_DIR="${REPO_DIR}/modules/${MODULE_NAME}/packages"

mkdir -p "${SRC_DIR}" "${PKG_DIR}"

# Link the packages we want in this module (modules/pilot/packages/pilot -> src/pilot)
if [ -d "${PKG_DIR}/${MODULE_NAME}" ]; then
  ln -sfn "${PKG_DIR}/${MODULE_NAME}" "${SRC_DIR}/${MODULE_NAME}"
else
  echo "[pilot/setup] Warning: module-local package not found: ${PKG_DIR}/${MODULE_NAME}" >&2
fi

# Include geometry_msgs dependency (part of ROS2 common interfaces)
# and any other core dependencies needed

# Check if websockets library is available
echo "[pilot/setup] Checking Python dependencies..."
if ! python3 -c 'import websockets' >/dev/null 2>&1; then
  echo "[pilot/setup] Installing websockets library (with --break-system-packages)..."
  if pip3 install --break-system-packages websockets >/dev/null 2>&1 || sudo pip3 install --break-system-packages websockets >/dev/null 2>&1; then
    echo "[pilot/setup] websockets installed successfully"
  else
    echo "[pilot/setup] Warning: Failed to install websockets. You may need to install it manually:"
    echo "[pilot/setup]   pip3 install --break-system-packages websockets"
  fi
else
  echo "[pilot/setup] websockets library already available"
fi

# Install system packages required to host an access point and provide DHCP/DNS
# Note: network access and apt availability are required. We try to be helpful
# but fall back to warnings if the install cannot be completed.
echo "[pilot/setup] Checking system packages for AP support..."
SUDO=""
if [ "$(id -u)" -ne 0 ]; then
  if command -v sudo >/dev/null 2>&1; then
    SUDO=sudo
  else
    echo "[pilot/setup] Warning: not running as root and sudo not available; cannot install system packages automatically"
  fi
fi

if command -v apt-get >/dev/null 2>&1; then
  PKGS=(hostapd dnsmasq iproute2 iw)
  echo "[pilot/setup] Detected apt-get. Will attempt to install: ${PKGS[*]}"
  if [ -n "$SUDO" ] || [ "$(id -u)" -eq 0 ]; then
    echo "[pilot/setup] Updating apt cache..."
    # Don't fail the whole script if update fails; continue with best-effort
    $SUDO apt-get update -y || echo "[pilot/setup] apt-get update failed (network may be unavailable)"
    echo "[pilot/setup] Installing packages: ${PKGS[*]}"
    if $SUDO apt-get install -y "${PKGS[@]}" >/dev/null 2>&1; then
      echo "[pilot/setup] System packages installed successfully"
    else
      echo "[pilot/setup] Warning: Failed to install some system packages. You may need to run as root or install manually:"
      echo "[pilot/setup]   sudo apt-get install -y ${PKGS[*]}"
    fi
  else
    echo "[pilot/setup] Warning: Cannot install system packages because sudo/root is not available"
    echo "[pilot/setup] Install manually: sudo apt-get install -y ${PKGS[*]}"
  fi
else
  echo "[pilot/setup] apt-get not found; automatic installation of hostapd/dnsmasq skipped. Install them manually on Debian-based systems:"
  echo "[pilot/setup]   sudo apt-get install -y hostapd dnsmasq iproute2 iw"
fi

# Ensure Python zeroconf is available for mDNS support
if ! python3 -c 'import zeroconf' >/dev/null 2>&1; then
  echo "[pilot/setup] Installing Python zeroconf package (with --break-system-packages)..."
  if pip3 install --break-system-packages zeroconf >/dev/null 2>&1 || sudo pip3 install --break-system-packages zeroconf >/dev/null 2>&1; then
    echo "[pilot/setup] zeroconf installed successfully"
  else
    echo "[pilot/setup] Warning: Failed to install zeroconf. You may need to install it manually:"
    echo "[pilot/setup]   pip3 install --break-system-packages zeroconf"
  fi
else
  echo "[pilot/setup] zeroconf package already available"
fi

echo "[pilot/setup] Done. Build with: make build"