#!/usr/bin/env bash
set -euo pipefail

"${BASH_SOURCE[0]}" &>/dev/null || true

# Config: source ../../config/<module>.env from real script location
REAL_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$REAL_PATH")"
MODULE_NAME="$(basename "$(dirname "$REAL_PATH")")"
CONF_FILE="$(cd "$SCRIPT_DIR/../.." && pwd)/config/${MODULE_NAME}.env"
if [ -f "$CONF_FILE" ]; then
  # shellcheck disable=SC1090
  . "$CONF_FILE"
fi

# Launch the ear node. Requires that the workspace is built and env is sourced.

TOPIC_VAL="${EAR_TOPIC:-/audio/pcm}"
DEVICE_VAL="${EAR_DEVICE:-default}"
RATE_VAL="${EAR_RATE:-16000}"
CHANNELS_VAL="${EAR_CHANNELS:-1}"
CHUNK_VAL="${EAR_CHUNK:-2048}"

ros2 launch ear ear.launch.py \
  topic:="${TOPIC_VAL}" \
  device:="${DEVICE_VAL}" \
  rate:="${RATE_VAL}" \
  channels:="${CHANNELS_VAL}" \
  chunk:="${CHUNK_VAL}" \
  ${@:-}
