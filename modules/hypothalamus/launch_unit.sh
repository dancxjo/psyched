#!/usr/bin/env bash
set -euo pipefail

SENSOR_TYPE=${HYPOTHALAMUS_SENSOR_TYPE:-DHT11}
GPIO_PIN=${HYPOTHALAMUS_GPIO_PIN:-D4}
POLL_INTERVAL=${HYPOTHALAMUS_POLL_INTERVAL:-2.5}
FRAME_ID=${HYPOTHALAMUS_FRAME_ID:-environment_link}
SIMULATE=${HYPOTHALAMUS_SIMULATE:-}
SIM_SEED=${HYPOTHALAMUS_SIMULATION_SEED:-}

ARGS=(
  --ros-args
  -p sensor_type:="${SENSOR_TYPE}"
  -p gpio_pin:="${GPIO_PIN}"
  -p poll_interval:=${POLL_INTERVAL}
  -p frame_id:="${FRAME_ID}"
)

if [[ -n "${SIMULATE}" ]]; then
  ARGS+=(-p simulate_when_unavailable:=${SIMULATE})
fi

if [[ -n "${SIM_SEED}" ]]; then
  ARGS+=(-p simulation_seed:=${SIM_SEED})
fi

ros2 run hypothalamus hypothalamus_node "${ARGS[@]}" &

wait -n
