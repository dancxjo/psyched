#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="${REPO_DIR:-$(cd "${SCRIPT_DIR}/../.." && pwd)}"
WORKSPACE_SRC="${PSYCHED_WORKSPACE_SRC:-${REPO_ROOT}/work/src}"

IMU_DIR="${WORKSPACE_SRC}/ros2_mpu6050_driver"
HEADER="${IMU_DIR}/include/mpu6050driver/mpu6050sensor.h"
CPP="${IMU_DIR}/src/mpu6050driver.cpp"
SENSOR_CPP="${IMU_DIR}/src/mpu6050sensor.cpp"
PARAMS_FILE="${IMU_DIR}/params/mpu6050.yaml"

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

if [[ ! -f "${SENSOR_CPP}" ]]; then
  echo "[imu] Source ${SENSOR_CPP} missing; cannot patch" >&2
  exit 1
fi

if [[ ! -f "${PARAMS_FILE}" ]]; then
  echo "[imu] Params file ${PARAMS_FILE} missing; cannot patch" >&2
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

export HEADER CPP SENSOR_CPP PARAMS_FILE

python3 <<'PY'
import os
import re
from pathlib import Path

header_path = Path(os.environ["HEADER"])
cpp_path = Path(os.environ["CPP"])
sensor_path = Path(os.environ["SENSOR_CPP"])
params_path = Path(os.environ["PARAMS_FILE"])

header_text = header_path.read_text()
updated_header = header_text
updated_header = updated_header.replace(
  'char filename_[10] = "/dev/i2c-";',
  'char filename_[16] = "/dev/i2c-";',
  1,
)
if updated_header != header_text:
  header_path.write_text(updated_header)
  print(f"[imu] Expanded I2C device buffer in {header_path}")
else:
  print(f"[imu] I2C device buffer size already updated in {header_path}")

cpp_text = cpp_path.read_text()
updated_cpp = cpp_text

constructor_prefix = (
  "MPU6050Driver::MPU6050Driver()\n"
  "    : Node(\"mpu6050publisher\"), mpu6050_{std::make_unique<MPU6050Sensor>()}\n"
  "{\n"
  "  // Declare parameters\n"
  "  declareParameters();\n"
)

if constructor_prefix in updated_cpp:
  updated_cpp = updated_cpp.replace(
    constructor_prefix,
    "MPU6050Driver::MPU6050Driver()\n"
    "    : Node(\"mpu6050publisher\")\n"
    "{\n"
    "  // Declare parameters\n"
    "  declareParameters();\n"
    "  const int bus_number = this->get_parameter(\"bus_number\").as_int();\n"
    "  mpu6050_ = std::make_unique<MPU6050Sensor>(bus_number);\n"
  )

decl_pattern = re.compile(
  r"void MPU6050Driver::declareParameters\(\)\n\{\n" +
  r"  this->declare_parameter<bool>\(\"calibrate\", true\);\n" +
  r"  this->declare_parameter<int>\(\"gyro_range\", MPU6050Sensor::GyroRange::GYR_250_DEG_S\);\n" +
  r"  this->declare_parameter<int>\(\"accel_range\", MPU6050Sensor::AccelRange::ACC_2_G\);\n" +
  r"  this->declare_parameter<int>\(\"dlpf_bandwidth\", MPU6050Sensor::DlpfBandwidth::DLPF_260_HZ\);\n" +
  r"  this->declare_parameter<double>\(\"gyro_x_offset\", 0.0\);\n" +
  r"  this->declare_parameter<double>\(\"gyro_y_offset\", 0.0\);\n" +
  r"  this->declare_parameter<double>\(\"gyro_z_offset\", 0.0\);\n" +
  r"  this->declare_parameter<double>\(\"accel_x_offset\", 0.0\);\n" +
  r"  this->declare_parameter<double>\(\"accel_y_offset\", 0.0\);\n" +
  r"  this->declare_parameter<double>\(\"accel_z_offset\", 0.0\);\n" +
  r"  this->declare_parameter<int>\(\"frequency\", 0.0\);\n" +
  r"}\n"
)

new_decl = (
  "void MPU6050Driver::declareParameters()\n"
  "{\n"
  "  this->declare_parameter<bool>(\"calibrate\", true);\n"
  "  this->declare_parameter<int>(\"gyro_range\", MPU6050Sensor::GyroRange::GYR_250_DEG_S);\n"
  "  this->declare_parameter<int>(\"accel_range\", MPU6050Sensor::AccelRange::ACC_2_G);\n"
  "  this->declare_parameter<int>(\"dlpf_bandwidth\", MPU6050Sensor::DlpfBandwidth::DLPF_260_HZ);\n"
  "  this->declare_parameter<double>(\"gyro_x_offset\", 0.0);\n"
  "  this->declare_parameter<double>(\"gyro_y_offset\", 0.0);\n"
  "  this->declare_parameter<double>(\"gyro_z_offset\", 0.0);\n"
  "  this->declare_parameter<double>(\"accel_x_offset\", 0.0);\n"
  "  this->declare_parameter<double>(\"accel_y_offset\", 0.0);\n"
  "  this->declare_parameter<double>(\"accel_z_offset\", 0.0);\n"
  "  this->declare_parameter<int>(\"frequency\", 0);\n"
  "  this->declare_parameter<int>(\"bus_number\", 1);\n"
  "}\n"
)

if decl_pattern.search(updated_cpp):
  updated_cpp = decl_pattern.sub(new_decl, updated_cpp, count=1)

if updated_cpp != cpp_text:
  cpp_path.write_text(updated_cpp)
  print(f"[imu] Applied bus parameter support patch to {cpp_path}")
else:
  print(f"[imu] Bus parameter support already present in {cpp_path}")

sensor_text = sensor_path.read_text()
updated_sensor = sensor_text

if "std::snprintf" not in updated_sensor:
  if "#include <cstdio>" not in updated_sensor:
    updated_sensor = updated_sensor.replace(
      "#include <iostream>\n",
      "#include <iostream>\n#include <cstdio>\n",
      1,
    )
  target_block = (
    "  // TODO: make char append cleaner\n"
    "  filename_[9] = *std::to_string(bus_number).c_str();\n"
    "  std::cout << filename_ << std::endl;\n"
  )
  replacement_block = (
    "  std::snprintf(filename_, sizeof(filename_), \"/dev/i2c-%d\", bus_number);\n"
    "  std::cout << filename_ << std::endl;\n"
  )
  if target_block in updated_sensor:
    updated_sensor = updated_sensor.replace(target_block, replacement_block, 1)

if updated_sensor != sensor_text:
  sensor_path.write_text(updated_sensor)
  print(f"[imu] Updated sensor bus path handling in {sensor_path}")
else:
  print(f"[imu] Sensor bus path already handled in {sensor_path}")

params_text = params_path.read_text().splitlines()
updated_lines = params_text[:]
for idx, line in enumerate(updated_lines):
  stripped = line.strip()
  if stripped.startswith("bus_number:"):
    indent = line[: len(line) - len(line.lstrip())]
    updated_lines[idx] = f"{indent}bus_number: 5"
    break
else:
  for idx, line in enumerate(updated_lines):
    if line.strip().startswith("frequency:"):
      indent = line[: len(line) - len(line.lstrip())]
      updated_lines.insert(idx + 1, f"{indent}bus_number: 5")
      break

if updated_lines != params_text:
  params_path.write_text("\n".join(updated_lines) + "\n")
  print(f"[imu] Set bus_number parameter to 5 in {params_path}")
else:
  print(f"[imu] bus_number parameter already set in {params_path}")

PY
