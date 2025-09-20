#!/usr/bin/env bash
set -euo pipefail

# Launch the pilot web interface node. Requires that the workspace is built and env is sourced.

# Resolve host config file
HOSTNAME_short="${HOST:-$(hostname -s)}"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CFG_FILE="${REPO_DIR}/hosts/${HOSTNAME_short}/config/pilot.toml"

cfg_get() {
  local key="$1"; local def_val="${2:-}"
  if [ -f "$CFG_FILE" ]; then
    local line
    line="$(grep -E "^${key}=" "$CFG_FILE" | tail -n1 || true)"
    if [ -n "$line" ]; then echo "${line#*=}"; return 0; fi
  fi
  echo "$def_val"
}

WEB_PORT_VAL="$(cfg_get web_port "${PILOT_WEB_PORT:-8080}")"
WEBSOCKET_PORT_VAL="$(cfg_get websocket_port "${PILOT_WS_PORT:-8081}")"
CMD_VEL_TOPIC_VAL="$(cfg_get cmd_vel_topic "${PILOT_CMD_VEL_TOPIC:-/cmd_vel}")"
HOST_VAL="$(cfg_get host "${PILOT_HOST:-0.0.0.0}")"

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