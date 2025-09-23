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

case "${ROS_DISTRO}" in
  jade|kinetic|lunar|melodic|noetic)
    echo "Skipping end-of-life ROS distro: ${ROS_DISTRO}"
    exit 0
    ;;
  jazzy|kilted|rolling)
    echo "Detected supported ROS2 distro: ${ROS_DISTRO}"
    ;;
  *)
    echo "Proceeding with generic ROS2 distro: ${ROS_DISTRO}"
    ;;
esac
# Determine repository root regardless of current working directory
if REPO_DIR_GIT_ROOT=$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel 2>/dev/null); then
  REPO_DIR="$REPO_DIR_GIT_ROOT"
else
  REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
fi
SOURCE_DIR="${REPO_DIR}/src"

MPU6050_REPO="https://github.com/hiwad-aziz/ros2_mpu6050_driver.git"

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

DRIVER_DIR_NAME="ros2_mpu6050_driver"
# Clone the requested driver repo if missing
if [ ! -d "${SOURCE_DIR}/${DRIVER_DIR_NAME}/.git" ]; then
  git clone "${MPU6050_REPO}" "${SOURCE_DIR}/${DRIVER_DIR_NAME}"
else
  echo "[imu/setup] ${DRIVER_DIR_NAME} already present; skipping clone"
fi

# Apply idempotent patch requested by user to add <array> include
PATCH_HEADER_PATH="${SOURCE_DIR}/${DRIVER_DIR_NAME}/include/mpu6050driver/mpu6050sensor.h"
if [ -f "${PATCH_HEADER_PATH}" ]; then
  # Check whether <array> is already present
  if ! grep -q "#include <array>" "${PATCH_HEADER_PATH}" 2>/dev/null; then
    echo "[imu/setup] Patching ${PATCH_HEADER_PATH}: inserting #include <array> after #include <string>"
    # Insert after the first occurrence of #include <string>
    if grep -q "#include <string>" "${PATCH_HEADER_PATH}" 2>/dev/null; then
      sed -i '/#include <string>/a #include <array>' "${PATCH_HEADER_PATH}"
    else
      # Fallback: add near top of file
      sed -i '1i #include <array>' "${PATCH_HEADER_PATH}"
    fi
  else
    echo "[imu/setup] ${PATCH_HEADER_PATH} already contains <array>; no change"
  fi
else
  echo "[imu/setup] ${PATCH_HEADER_PATH} not found; skipping header patch"
fi

# Ensure I2C development headers are available for build (Ubuntu/Debian)
if command -v apt >/dev/null 2>&1; then
  echo "[imu/setup] Installing i2c headers (libi2c-dev) and tools..."
  sudo apt update && sudo apt install -y libi2c-dev i2c-tools || true
fi

echo "IMU module setup complete. Now run: make build"
