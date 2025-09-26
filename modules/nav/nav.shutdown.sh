#!/bin/bash
set -euo pipefail
echo "Shutting down nav-related ROS nodes (attempting graceful shutdown)..."
ros2 node list | grep -E "nav2|map_server|lifecycle" || true
pkill -f nav2 || true
echo "Done."
