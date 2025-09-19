#!/usr/bin/env bash
set -euo pipefail

"${BASH_SOURCE[0]}" &>/dev/null || true

# Launch the ear node. Requires that the workspace is built and env is sourced.

# Resolve host config file
HOSTNAME_short="${HOST:-$(hostname -s)}"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CFG_FILE="${REPO_DIR}/hosts/${HOSTNAME_short}/config/ear.toml"

# Minimal parser for simple key=value lines
cfg_get() {
  local key="$1"
  local def_val="${2:-}"
  if [ -f "$CFG_FILE" ]; then
    local line
    line="$(grep -E "^${key}=" "$CFG_FILE" | tail -n1 || true)"
    if [ -n "$line" ]; then
      echo "${line#*=}"
      return 0
    fi
  fi
  echo "$def_val"
}

TOPIC_VAL="$(cfg_get topic "/audio/pcm")"
DEVICE_VAL="$(cfg_get device "default")"
RATE_VAL="$(cfg_get rate "16000")"
CHANNELS_VAL="$(cfg_get channels "1")"
CHUNK_VAL="$(cfg_get chunk "2048")"

ros2 launch ear ear.launch.py \
  topic:="${TOPIC_VAL}" \
  device:="${DEVICE_VAL}" \
  rate:="${RATE_VAL}" \
  channels:="${CHANNELS_VAL}" \
  chunk:="${CHUNK_VAL}" \
  ${@:-}
