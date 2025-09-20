#!/usr/bin/env bash
set -euo pipefail

# Config: source ../../config/<module>.env from real script location
REAL_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$REAL_PATH")"
MODULE_NAME="$(basename "$(dirname "$REAL_PATH")")"
CONF_FILE="$(cd "$SCRIPT_DIR/../.." && pwd)/config/${MODULE_NAME}.env"
if [ -f "$CONF_FILE" ]; then
  # shellcheck disable=SC1090
  . "$CONF_FILE"
fi

TOPIC_VAL="${FOOT_TOPIC:-/foot}"

# Export for any nodes that may read env
export FOOT_TOPIC="$TOPIC_VAL"

# If the create_bringup supports remappings/params, we could pass them here.
ros2 launch create_bringup create_1.launch
