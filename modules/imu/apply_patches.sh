#!/bin/bash
set -euo pipefail
ROOT="${REPO_DIR:-$(cd "$(dirname "$0")/../.." && pwd)}"

# Ensure <array> is included in the header
HEADER="$ROOT/src/ros2_mpu6050_driver/include/mpu6050driver/mpu6050sensor.h"
if [ -f "$HEADER" ] && ! grep -q '#include <array>' "$HEADER"; then
  sed -i '/#include <string>/a #include <array>' "$HEADER" || sed -i '1i #include <array>' "$HEADER"
fi

# Fix publisher topic to imu/data_raw
CPP="$ROOT/src/ros2_mpu6050_driver/src/mpu6050driver.cpp"
if [ -f "$CPP" ]; then
  sed -i 's|\(publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(\)"imu"\(, *10);\)|\1"imu/data_raw"\2|' "$CPP"
fi
