#!/usr/bin/env bash
set -euo pipefail

# Config: source ../../config/imu.env from real script location
REAL_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$REAL_PATH")"
MODULE_NAME="$(basename "$(dirname "$REAL_PATH")")"
CONF_FILE="$(cd "$SCRIPT_DIR/../.." && pwd)/config/${MODULE_NAME}.env"
if [ -f "$CONF_FILE" ]; then
  # shellcheck disable=SC1090
  . "$CONF_FILE"
fi

# Determine repository root regardless of current working directory
if REPO_DIR_GIT_ROOT=$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel 2>/dev/null); then
  REPO_DIR="$REPO_DIR_GIT_ROOT"
else
  REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
fi
SOURCE_DIR="${REPO_DIR}/src"

MPU6050_REPO="https://github.com/kimsniper/ros2_mpu6050.git"

if ! command -v git >/dev/null 2>&1; then
  echo "Error: 'git' is required. Install Git from https://git-scm.com/downloads" >&2
  exit 1
fi

mkdir -p "${SOURCE_DIR}"
echo "[imu/setup] Using repo root: ${REPO_DIR}"
echo "[imu/setup] Populating source dir: ${SOURCE_DIR}"

# Avoid duplicate package copies if someone previously cloned under the module dir
LOCAL_MODULE_SRC_DIR="${SCRIPT_DIR}/src"
if [ -d "${LOCAL_MODULE_SRC_DIR}" ]; then
  if [ -d "${LOCAL_MODULE_SRC_DIR}/ros2_mpu6050" ]; then
    echo "[imu/setup] Removing stale module-local clone: ${LOCAL_MODULE_SRC_DIR}/ros2_mpu6050"
    rm -rf "${LOCAL_MODULE_SRC_DIR}/ros2_mpu6050"
  fi
  rmdir "${LOCAL_MODULE_SRC_DIR}" 2>/dev/null || true
fi

# Clone ros2_mpu6050 if missing
if [ ! -d "${SOURCE_DIR}/ros2_mpu6050/.git" ]; then
  git clone "${MPU6050_REPO}" "${SOURCE_DIR}/ros2_mpu6050"
else
  echo "[imu/setup] ros2_mpu6050 already present; skipping clone"
fi

# Ensure I2C development headers are available for build (Ubuntu/Debian)
if command -v apt >/dev/null 2>&1; then
  echo "[imu/setup] Installing i2c headers (libi2c-dev) and tools..."
  sudo apt update && sudo apt install -y libi2c-dev i2c-tools || true
fi

echo "IMU module setup complete. Now run: make build"
