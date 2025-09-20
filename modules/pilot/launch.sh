#!/usr/bin/env bash
set -euo pipefail

# Launch the pilot web interface node. Requires that the workspace is built and env is sourced.

HOSTNAME_short="${HOST:-$(hostname -s)}"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ENV_FILE="${REPO_DIR}/hosts/${HOSTNAME_short}/config/pilot.env"

if [ -f "$ENV_FILE" ]; then
  # shellcheck disable=SC1090
  . "$ENV_FILE"
fi

WEB_PORT_VAL="${PILOT_WEB_PORT:-8080}"
WEBSOCKET_PORT_VAL="${PILOT_WS_PORT:-8081}"
CMD_VEL_TOPIC_VAL="${PILOT_CMD_VEL_TOPIC:-/cmd_vel}"
HOST_VAL="${PILOT_HOST:-0.0.0.0}"

# Export for any nodes that may read env
export PILOT_WEB_PORT="$WEB_PORT_VAL"
export PILOT_WS_PORT="$WEBSOCKET_PORT_VAL"
export PILOT_CMD_VEL_TOPIC="$CMD_VEL_TOPIC_VAL"
export PILOT_HOST="$HOST_VAL"

echo "[pilot/launch] Starting pilot web interface..."
echo "[pilot/launch] Web: http://${HOST_VAL}:${WEB_PORT_VAL}"
echo "[pilot/launch] WebSocket: ws://${HOST_VAL}:${WEBSOCKET_PORT_VAL}"
echo "[pilot/launch] Publishing to: ${CMD_VEL_TOPIC_VAL}"

ros2 launch pilot pilot.launch.py \
  web_port:="${WEB_PORT_VAL}" \
  websocket_port:="${WEBSOCKET_PORT_VAL}" \
  cmd_vel_topic:="${CMD_VEL_TOPIC_VAL}" \
  host:="${HOST_VAL}" \
  ${@:-}