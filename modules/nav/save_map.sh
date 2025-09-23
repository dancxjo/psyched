#!/usr/bin/env bash
set -euo pipefail

OUT=${1:-nav_map}
echo "Exporting map to ${OUT}.pgm/.yaml using Nav2 map_saver_cli (if available)"

if command -v ros2 >/dev/null 2>&1; then
  ros2 run nav2_map_server map_saver_cli -f "${OUT}" || {
    echo "map_saver_cli failed or not available; please save map using your preferred tool."
    exit 1
  }
else
  echo "ros2 not found — cannot save map."
  exit 1
fi
