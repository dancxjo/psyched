#!/usr/bin/env bash
set -euo pipefail

PATTERN="ros2 run voice speech_service"

if pgrep -f "$PATTERN" >/dev/null 2>&1; then
  echo "[voice/shutdown] Stopping $PATTERN"
  pkill -TERM -f "$PATTERN" || true
fi

for _ in {1..10}; do
  if ! pgrep -f "$PATTERN" >/dev/null 2>&1; then
    echo "[voice/shutdown] Voice service stopped"
    exit 0
  fi
  sleep 1
done

pkill -KILL -f "$PATTERN" || true
echo "[voice/shutdown] Forced kill"
