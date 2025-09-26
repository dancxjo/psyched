#!/bin/bash
set -euo pipefail
HEADER="src/ros2_mpu6050_driver/include/mpu6050driver/mpu6050sensor.h"
if [ -f "$HEADER" ] && ! grep -q '#include <array>' "$HEADER"; then
  if grep -q '#include <string>' "$HEADER"; then
    sed -i '/#include <string>/a #include <array>' "$HEADER"
  else
    sed -i '1i #include <array>' "$HEADER"
  fi
fi
