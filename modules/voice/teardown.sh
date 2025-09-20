#!/usr/bin/env bash
set -euo pipefail

# Config: source ../../config/voice.env from real script location
REAL_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$REAL_PATH")"
MODULE_NAME="$(basename "$(dirname "$REAL_PATH")")"
CONF_FILE="$(cd "$SCRIPT_DIR/../.." && pwd)/config/${MODULE_NAME}.env"
if [ -f "$CONF_FILE" ]; then
    # shellcheck disable=SC1090
    . "$CONF_FILE"
fi

# Shared module helpers
MODULE_LIB="$(cd "$SCRIPT_DIR/../.." && pwd)/tools/lib/module.sh"
if [ -f "$MODULE_LIB" ]; then
    # shellcheck disable=SC1090
    . "$MODULE_LIB"
fi

REPO_DIR="$(pwd)"
SERVICE_NAME="psyched-voice.service"
psh_remove_artifacts "$REPO_DIR"
psh_remove_src_entries "$REPO_DIR" voice psyched
psh_systemd_disable "$SERVICE_NAME"
