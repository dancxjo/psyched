#!/bin/bash
set -euo pipefail
TOPIC_VAL="${EAR_TOPIC:-/audio/pcm}"
DEVICE_VAL="${EAR_DEVICE:-default}"
RATE_VAL="${EAR_RATE:-16000}"
CHANNELS_VAL="${EAR_CHANNELS:-1}"
CHUNK_VAL="${EAR_CHUNK:-2048}"

REPO_DIR="${REPO_DIR:-$(cd "$(dirname "$0")/../.." && pwd)}"
export REPO_DIR
exec ros2 launch ear ear.launch.py \
  topic:="${TOPIC_VAL}" \
  device:="${DEVICE_VAL}" \
  rate:="${RATE_VAL}" \
  channels:="${CHANNELS_VAL}" \
  chunk:="${CHUNK_VAL}"
