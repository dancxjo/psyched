#!/usr/bin/env bash
set -euo pipefail

# Config: source ../../config/<module>.env from real script location
REAL_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$REAL_PATH")"
MODULE_NAME="$(basename "$(dirname "$REAL_PATH")")"
CONF_FILE="$(cd "$SCRIPT_DIR/../.." && pwd)/config/${MODULE_NAME}.env"
if [ -f "$CONF_FILE" ]; then
  # shellcheck disable=SC1090
  . "$CONF_FILE"
fi

# Optional param file override; default uses package share config/params.yaml
PARAM_FILE_VAL="${IMU_PARAM_FILE:-}" # renamed from MPU6050_PARAM_FILE

LAUNCH_ARGS=()
if [ -n "$PARAM_FILE_VAL" ]; then
  LAUNCH_ARGS+=("param_file:=${PARAM_FILE_VAL}")
fi

ros2 launch mpu6050driver mpu6050driver_launch.py ${LAUNCH_ARGS[@]:-}
