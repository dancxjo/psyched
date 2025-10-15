#!/usr/bin/env bash
set -euo pipefail

PATTERN="ros2 launch ear ear.launch.py"

if pgrep -f "$PATTERN" >/dev/null 2>&1; then
  echo "[ear/shutdown] Stopping $PATTERN"
  pkill -TERM -f "$PATTERN" || true
fi

for _ in {1..10}; do
  if ! pgrep -f "$PATTERN" >/dev/null 2>&1; then
    echo "[ear/shutdown] Ear service stopped"
    exit 0
  fi
  sleep 1
done

pkill -KILL -f "$PATTERN" || true
echo "[ear/shutdown] Forced kill"
