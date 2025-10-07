#!/usr/bin/env bash
set -euo pipefail

PATTERN="ros2 run viscera viscera_monitor"

if pgrep -f "$PATTERN" >/dev/null 2>&1; then
  echo "[viscera/shutdown] Stopping $PATTERN"
  pkill -TERM -f "$PATTERN" || true
fi

for _ in {1..10}; do
  if ! pgrep -f "$PATTERN" >/dev/null 2>&1; then
    echo "[viscera/shutdown] Viscera monitor stopped"
    exit 0
  fi
  sleep 1
fi

pkill -KILL -f "$PATTERN" || true
echo "[viscera/shutdown] Forced kill"
