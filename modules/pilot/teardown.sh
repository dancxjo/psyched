#!/usr/bin/env bash
set -euo pipefail

# Teardown the pilot module - stop any running processes

echo "[pilot/teardown] Stopping pilot web interface..."

# Kill any running pilot nodes
pkill -f "pilot_node" || true

# Config: source ../../config/<module>.env and module helpers
REAL_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$REAL_PATH")"
MODULE_NAME="$(basename "$(dirname "$REAL_PATH")")"
CONF_FILE="$(cd "$SCRIPT_DIR/../.." && pwd)/config/${MODULE_NAME}.env"
if [ -f "$CONF_FILE" ]; then
  # shellcheck disable=SC1090
  . "$CONF_FILE"
fi
PILOT_WEB_PORT="${PILOT_WEB_PORT:-8080}"
PILOT_WS_PORT="${PILOT_WS_PORT:-8081}"

for port in "$PILOT_WEB_PORT" "$PILOT_WS_PORT"; do
  PID=$(lsof -ti:$port 2>/dev/null || true)
  if [ -n "$PID" ]; then
    echo "[pilot/teardown] Stopping process on port $port (PID: $PID)"
    kill "$PID" 2>/dev/null || true
  fi
done

echo "[pilot/teardown] Done."