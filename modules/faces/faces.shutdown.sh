#!/bin/bash
set -euo pipefail
echo "Stopping Psyched face detector node..."
ros2 node list | grep -E "psyched_faces" || true
pkill -f face_detector || true
echo "Done."
