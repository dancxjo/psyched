#!/bin/bash
set -euo pipefail

HEADER="src/ros2_mpu6050_driver/src/mpu6050driver.cpp"

if [ -f "$HEADER" ]; then
  # Replace any publisher_ line that uses "imu" with "imu/data_raw"
  sed -i 's|\(publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(\)"imu"\(, *10);\)|\1"imu/data_raw"\2|' "$HEADER"
fi
