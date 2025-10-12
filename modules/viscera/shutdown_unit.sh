#!/usr/bin/env bash
set -euo pipefail

PATTERNS=(
  "ros2 run viscera viscera_monitor"
  "ros2 run viscera viscera_host_health"
)

for pattern in "${PATTERNS[@]}"; do
  if pgrep -f "$pattern" >/dev/null 2>&1; then
    echo "[viscera/shutdown] Stopping $pattern"
    pkill -TERM -f "$pattern" || true
  fi
done

for _ in {1..10}; do
  remaining=0
  for pattern in "${PATTERNS[@]}"; do
    if pgrep -f "$pattern" >/dev/null 2>&1; then
      remaining=$((remaining + 1))
    fi
  done
  if [[ $remaining -eq 0 ]]; then
    echo "[viscera/shutdown] Viscera processes stopped"
    exit 0
  fi
  sleep 1
done

for pattern in "${PATTERNS[@]}"; do
  pkill -KILL -f "$pattern" || true
done
echo "[viscera/shutdown] Forced kill"
