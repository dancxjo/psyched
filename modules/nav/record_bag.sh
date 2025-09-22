#!/usr/bin/env bash
set -euo pipefail

OUT=${1:-nav_record}
echo "Recording topics to ${OUT}.bag — press Ctrl-C to stop"

if command -v ros2 >/dev/null 2>&1; then
  ros2 bag record -o "${OUT}" /tf /tf_static /camera/color/image_raw /camera/depth/image_raw /odom
else
  echo "ros2 not found — cannot record bag."
  exit 1
fi
