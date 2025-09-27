#!/bin/bash
set -euo pipefail
REPO_DIR="${REPO_DIR:-$(cd "$(dirname "$0")/../.." && pwd)}"
export REPO_DIR
HOST_SHORT="${HOST:-$(hostname -s)}"

HOST_YAML="${REPO_DIR}/hosts/${HOST_SHORT}/config/imu.yaml"

if [ -f "$HOST_YAML" ]; then
  exec ros2 launch mpu6050driver mpu6050driver_launch.py --ros-args --params-file "$HOST_YAML"
else
  exec ros2 launch mpu6050driver mpu6050driver_launch.py
fi

# Start Madgwick filter
ros2 run imu_filter_madgwick imu_filter_madgwick_node \
  --ros-args \
    -p use_mag:=false \
    -p world_frame:="enu" \
    -p publish_tf:=false \
    -p gain:=0.1 &
    
wait -n