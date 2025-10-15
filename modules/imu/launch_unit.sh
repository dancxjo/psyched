#!/bin/bash
set -euo pipefail

# Launch mpu6050 driver in background
ros2 launch mpu6050driver mpu6050driver_launch.py &

# Launch Madgwick filter as main process
exec ros2 run imu_filter_madgwick imu_filter_madgwick_node \
  --ros-args \
    -p use_mag:=false \
    -p world_frame:="enu" \
    -p publish_tf:=false \
    -p gain:=0.1
