#!/usr/bin/env bash
set -euo pipefail

# Teardown the pilot module - stop any running processes

echo "[pilot/teardown] Stopping pilot web interface..."

# Kill any running pilot nodes
pkill -f "pilot_node" || true

# Kill any processes using the pilot ports
HOSTNAME_short="${HOST:-$(hostname -s)}"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ENV_FILE="${REPO_DIR}/hosts/${HOSTNAME_short}/config/pilot.env"
if [ -f "$ENV_FILE" ]; then
  # shellcheck disable=SC1090
  . "$ENV_FILE"
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