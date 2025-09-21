#!/usr/bin/env bash
set -euo pipefail

# Config: source ../../config/imu.env from real script location
REAL_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$REAL_PATH")"
MODULE_NAME="$(basename "$(dirname "$REAL_PATH")")"
CONF_FILE="$(cd "$SCRIPT_DIR/../.." && pwd)/config/${MODULE_NAME}.env"
if [ -f "$CONF_FILE" ]; then
    # shellcheck disable=SC1090
    . "$CONF_FILE"
fi

REPO_DIR="$(pwd)"
SERVICE_NAME="psyched-imu.service"
# Remove common build artifacts
for a in build install log; do
    [ -d "${REPO_DIR}/$a" ] && rm -rf "${REPO_DIR}/$a"
done

# Remove specific src entries
SRC_DIR="${REPO_DIR}/src"
# remove either the old nested name or the new driver directory
for d in ros2_mpu6050 ros2_mpu6050_driver; do
    if [ -L "${SRC_DIR}/$d" ] || [ -d "${SRC_DIR}/$d" ]; then
        rm -rf "${SRC_DIR}/$d"
    fi
done

# Disable/remove systemd service if possible
if command -v systemctl >/dev/null 2>&1; then
    systemctl disable --now "$SERVICE_NAME" >/dev/null 2>&1 || true
fi
SYSTEMD_DIR="${SYSTEMD_DIR:-/etc/systemd/system}"
if [ -f "${SYSTEMD_DIR}/${SERVICE_NAME}" ]; then
    if command -v sudo >/dev/null 2>&1; then
        sudo rm -f "${SYSTEMD_DIR}/${SERVICE_NAME}" || true
    else
        rm -f "${SYSTEMD_DIR}/${SERVICE_NAME}" || true
    fi
fi
