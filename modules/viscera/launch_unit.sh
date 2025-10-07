#!/usr/bin/env bash
set -euo pipefail

INTERVAL=${VISCERA_INTERVAL:-5.0}
LIMIT_ARGS=()
if [[ -n "${VISCERA_SENTIMENT_LIMIT:-}" ]]; then
  LIMIT_ARGS=("--limit" "${VISCERA_SENTIMENT_LIMIT}")
fi

ros2 run viscera viscera_monitor -- --interval "${INTERVAL}" "${LIMIT_ARGS[@]}" &

wait -n
