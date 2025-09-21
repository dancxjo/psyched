#!/usr/bin/env bash
set -euo pipefail

# Config: source ../../config/gps.env from real script location
REAL_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$REAL_PATH")"
MODULE_NAME="$(basename "$(dirname "$REAL_PATH")")"
CONF_FILE="$(cd "$SCRIPT_DIR/../.." && pwd)/config/${MODULE_NAME}.env"
if [ -f "$CONF_FILE" ]; then
  # shellcheck disable=SC1090
  . "$CONF_FILE"
fi

# Launch the u-blox GPS node (requires env sourced)
FRAME_ID_VAL="${GPS_FRAME_ID:-gps_link}"
DEVICE_VAL="${GPS_DEVICE:-/dev/gps0}"
PUBLISH_RATE_VAL="${GPS_PUBLISH_RATE:-5.0}"

ros2 launch ublox_gps ublox_gps.launch.py \
  frame_id:="${FRAME_ID_VAL}" \
  device:="${DEVICE_VAL}" \
  publish_rate:="${PUBLISH_RATE_VAL}" \
  ${@:-}
