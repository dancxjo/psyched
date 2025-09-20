#!/usr/bin/env bash
set -euo pipefail

# Config: source ../../config/chat.env from real script location
REAL_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$REAL_PATH")"
MODULE_NAME="$(basename "$(dirname "$REAL_PATH")")"
CONF_FILE="$(cd "$SCRIPT_DIR/../.." && pwd)/config/${MODULE_NAME}.env"
if [ -f "$CONF_FILE" ]; then
	# shellcheck disable=SC1090
	. "$CONF_FILE"
fi

echo "[chat/teardown] Nothing to teardown currently."
