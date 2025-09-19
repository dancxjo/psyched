#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(pwd)"
SERVICE_NAME="psyched-foot.service"
SYSTEMD_DIR="${PSH_SYSTEMD_DIR:-/etc/systemd/system}"
SERVICE_PATH="${SYSTEMD_DIR}/${SERVICE_NAME}"

for artifact in build install log; do
    if [ -d "${REPO_DIR}/${artifact}" ]; then
        rm -rf "${REPO_DIR}/${artifact}"
        echo "Removed ${REPO_DIR}/${artifact}"
    fi
done

SOURCE_DIR="${REPO_DIR}/src"
for dir in create_robot libcreate; do
    if [ -d "${SOURCE_DIR}/${dir}" ]; then
        rm -rf "${SOURCE_DIR}/${dir}"
        echo "Removed ${SOURCE_DIR}/${dir}"
    fi
done

if command -v systemctl >/dev/null 2>&1; then
    systemctl disable --now "${SERVICE_NAME}" >/dev/null 2>&1 || true
fi

if [ -f "${SERVICE_PATH}" ]; then
    rm -f "${SERVICE_PATH}"
    echo "Removed ${SERVICE_PATH}"
fi
