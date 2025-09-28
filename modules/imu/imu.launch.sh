#!/bin/bash
set -euo pipefail
REPO_DIR="${REPO_DIR:-$(cd "$(dirname "$0")/../.." && pwd)}"
export REPO_DIR
HOST_SHORT="${HOST:-$(hostname -s)}"

exec ros2 launch mpu6050driver mpu6050driver_launch.py &

# Start Madgwick filter
ros2 run imu_filter_madgwick imu_filter_madgwick_node \
  --ros-args \
    -p use_mag:=false \
    -p world_frame:="enu" \
    -p publish_tf:=false \
    -p gain:=0.1 &
    
wait -n