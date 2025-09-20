#!/usr/bin/env bash
set -euo pipefail

"${BASH_SOURCE[0]}" &>/dev/null || true

# Launch the ear node. Requires that the workspace is built and env is sourced.

HOSTNAME_short="${HOST:-$(hostname -s)}"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ENV_FILE="${REPO_DIR}/hosts/${HOSTNAME_short}/config/ear.env"

if [ -f "$ENV_FILE" ]; then
  # shellcheck disable=SC1090
  . "$ENV_FILE"
fi

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
