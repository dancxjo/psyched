#!/usr/bin/env bash
set -euo pipefail

INTERVAL=${VISCERA_INTERVAL:-5.0}
HEALTH_INTERVAL=${VISCERA_HEALTH_INTERVAL:-$INTERVAL}
LIMIT_ARGS=()
if [[ -n "${VISCERA_SENTIMENT_LIMIT:-}" ]]; then
  LIMIT_ARGS=("--limit" "${VISCERA_SENTIMENT_LIMIT}")
fi

cleanup() {
  local code=$?
  if [[ -n "${MONITOR_PID:-}" ]] && kill -0 "${MONITOR_PID}" 2>/dev/null; then
    kill "${MONITOR_PID}" 2>/dev/null || true
  fi
  if [[ -n "${HEALTH_PID:-}" ]] && kill -0 "${HEALTH_PID}" 2>/dev/null; then
    kill "${HEALTH_PID}" 2>/dev/null || true
  fi
  wait || true
  exit "${code}"
}
trap cleanup EXIT

ros2 run viscera viscera_host_health -- --interval "${HEALTH_INTERVAL}" &
HEALTH_PID=$!

ros2 run viscera viscera_monitor -- --interval "${INTERVAL}" "${LIMIT_ARGS[@]}" &
MONITOR_PID=$!

wait -n
