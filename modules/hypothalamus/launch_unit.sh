#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="${REPO_DIR:-$(cd "${SCRIPT_DIR}/../.." && pwd)}"
export REPO_DIR

if [[ -f "${REPO_DIR}/work/install/setup.bash" ]]; then
  set +u
  # shellcheck disable=SC1091
  source "${REPO_DIR}/work/install/setup.bash"
  set -u
fi

if [[ -f "${REPO_DIR}/work/install/hypothalamus/share/hypothalamus/local_setup.bash" ]]; then
  set +u
  # shellcheck disable=SC1091
  source "${REPO_DIR}/work/install/hypothalamus/share/hypothalamus/local_setup.bash"
  set -u
fi

SENSOR_TYPE=${HYPOTHALAMUS_SENSOR_TYPE:-DHT11}
GPIO_PIN=${HYPOTHALAMUS_GPIO_PIN:-D4}
POLL_INTERVAL=${HYPOTHALAMUS_POLL_INTERVAL:-2.5}
FRAME_ID=${HYPOTHALAMUS_FRAME_ID:-environment_link}
SIMULATE=${HYPOTHALAMUS_SIMULATE:-}
SIM_SEED=${HYPOTHALAMUS_SIMULATION_SEED:-}
TEMP_TOPIC=${HYPOTHALAMUS_TEMPERATURE_TOPIC:-}
TEMP_F_TOPIC=${HYPOTHALAMUS_TEMPERATURE_FAHRENHEIT_TOPIC:-}
HUMIDITY_TOPIC=${HYPOTHALAMUS_HUMIDITY_TOPIC:-}
HUMIDITY_PERCENT_TOPIC=${HYPOTHALAMUS_HUMIDITY_PERCENT_TOPIC:-}
STATUS_TOPIC=${HYPOTHALAMUS_STATUS_TOPIC:-}

LAUNCH_ARGS=(
  sensor_type:="${SENSOR_TYPE}"
  gpio_pin:="${GPIO_PIN}"
  poll_interval:="${POLL_INTERVAL}"
  frame_id:="${FRAME_ID}"
)

if [[ -n "${SIMULATE}" ]]; then
  LAUNCH_ARGS+=(simulate_when_unavailable:="${SIMULATE}")
fi

if [[ -n "${SIM_SEED}" ]]; then
  LAUNCH_ARGS+=(simulation_seed:="${SIM_SEED}")
fi

if [[ -n "${TEMP_TOPIC}" ]]; then
  LAUNCH_ARGS+=(temperature_topic:="${TEMP_TOPIC}")
fi

if [[ -n "${TEMP_F_TOPIC}" ]]; then
  LAUNCH_ARGS+=(temperature_fahrenheit_topic:="${TEMP_F_TOPIC}")
fi

if [[ -n "${HUMIDITY_TOPIC}" ]]; then
  LAUNCH_ARGS+=(humidity_topic:="${HUMIDITY_TOPIC}")
fi

if [[ -n "${HUMIDITY_PERCENT_TOPIC}" ]]; then
  LAUNCH_ARGS+=(humidity_percent_topic:="${HUMIDITY_PERCENT_TOPIC}")
fi

if [[ -n "${STATUS_TOPIC}" ]]; then
  LAUNCH_ARGS+=(status_topic:="${STATUS_TOPIC}")
fi

exec ros2 launch hypothalamus hypothalamus.launch.py "${LAUNCH_ARGS[@]}"
