#!/usr/bin/env bash
set -euo pipefail

OUT=${1:-rtabmap_map}
echo "Saving RTAB-Map database and exporting map to ${OUT}.pgm/.yaml"

# Save map via rtabmap's map saver node if available
if command -v ros2 >/dev/null 2>&1; then
  ros2 run nav2_map_server map_saver_cli -f "${OUT}"
else
  echo "ros2 not found — cannot save map."
  exit 1
fi
