#!/bin/bash
set -euo pipefail
PARAM_FILE_VAL="${IMU_PARAM_FILE:-}"
if [ -n "$PARAM_FILE_VAL" ]; then
  exec ros2 launch mpu6050driver mpu6050driver_launch.py param_file:="${PARAM_FILE_VAL}"
else
  exec ros2 launch mpu6050driver mpu6050driver_launch.py
fi

REPO_DIR="${REPO_DIR:-$(cd "$(dirname "$0")/../.." && pwd)}"
export REPO_DIR
