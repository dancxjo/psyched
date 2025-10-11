#!/usr/bin/env bash
set -euo pipefail

PATTERN="ros2 run pilot cockpit"

if pgrep -f "$PATTERN" >/dev/null 2>&1; then
  echo "[pilot/shutdown] Stopping $PATTERN"
  pkill -TERM -f "$PATTERN" || true
fi

for _ in {1..10}; do
  if ! pgrep -f "$PATTERN" >/dev/null 2>&1; then
    echo "[pilot/shutdown] Cockpit server stopped"
    exit 0
  fi
  sleep 1
done

pkill -KILL -f "$PATTERN" || true
echo "[pilot/shutdown] Forced kill"
