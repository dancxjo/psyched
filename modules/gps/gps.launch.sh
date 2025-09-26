#!/bin/bash
set -euo pipefail
FRAME_ID_VAL="${GPS_FRAME_ID:-gps_link}"
DEVICE_VAL="${GPS_DEVICE:-/dev/gps0}"
PUBLISH_RATE_VAL="${GPS_PUBLISH_RATE:-5.0}"
exec ros2 launch ublox_gps ublox_gps.launch.py \
  frame_id:="${FRAME_ID_VAL}" \
  device:="${DEVICE_VAL}" \
  publish_rate:="${PUBLISH_RATE_VAL}"
