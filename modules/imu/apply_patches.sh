#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="${REPO_DIR:-$(cd "${SCRIPT_DIR}/../.." && pwd)}"
WORKSPACE_SRC="${PSYCHED_WORKSPACE_SRC:-${REPO_ROOT}/work/src}"

IMU_DIR="${WORKSPACE_SRC}/ros2_mpu6050_driver"
HEADER="${IMU_DIR}/include/mpu6050driver/mpu6050sensor.h"
CPP="${IMU_DIR}/src/mpu6050driver.cpp"

if [[ ! -d "${IMU_DIR}" ]]; then
  echo "[imu] Expected directory ${IMU_DIR} not found; has the repository been synced?" >&2
  exit 1
fi

if [[ ! -f "${HEADER}" ]]; then
  echo "[imu] Header ${HEADER} missing; cannot patch" >&2
  exit 1
fi

if [[ ! -f "${CPP}" ]]; then
  echo "[imu] Source ${CPP} missing; cannot patch" >&2
  exit 1
fi

if ! grep -q '#include <array>' "${HEADER}"; then
  if ! sed -i '/#include <string>/a #include <array>' "${HEADER}"; then
    sed -i '1i #include <array>' "${HEADER}"
  fi
  echo "[imu] Added <array> include to ${HEADER}"
else
  echo "[imu] <array> include already present in ${HEADER}"
fi

if grep -q '"imu"' "${CPP}"; then
  sed -i 's|\(publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(\)"imu"\(, *10);\)|\1"imu/data_raw"\2|' "${CPP}"
  echo "[imu] Updated publisher topic to imu/data_raw in ${CPP}"
else
  echo "[imu] Publisher topic already patched in ${CPP}"
fi
