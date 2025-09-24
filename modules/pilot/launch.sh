#!/usr/bin/env bash
set -euo pipefail

# Config: source ../../config/<module>.env from real script location
REAL_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$REAL_PATH")"
MODULE_NAME="$(basename "$(dirname "$REAL_PATH")")"
CONF_FILE="$(cd "$SCRIPT_DIR/../.." && pwd)/config/${MODULE_NAME}.env"
if [ -f "$CONF_FILE" ]; then
  # shellcheck disable=SC1090
  . "$CONF_FILE"
fi

# Launch the pilot web interface node. Requires that the workspace is built and env is sourced.

WEB_PORT_VAL="${PILOT_WEB_PORT:-8080}"
WEBSOCKET_PORT_VAL="${PILOT_WS_PORT:-8081}"
CMD_VEL_TOPIC_VAL="${PILOT_CMD_VEL_TOPIC:-/cmd_vel}"
HOST_VAL="${PILOT_HOST:-0.0.0.0}"
VOICE_TOPIC_VAL="${PILOT_VOICE_TOPIC:-/voice}"
GPS_FIX_TOPIC_VAL="${PILOT_GPS_FIX_TOPIC:-/gps/fix}"
CONV_TOPIC_VAL="${PILOT_CONVERSATION_TOPIC:-/conversation}"
ENABLE_HTTP_VAL="${PILOT_ENABLE_HTTP:-true}"
ENABLE_WS_VAL="${PILOT_ENABLE_WS:-false}"
RUN_SEPARATE_WS_VAL="${PILOT_RUN_SEPARATE_WS:-true}"
ENABLE_AP_VAL="${PILOT_ENABLE_AP:-true}"
AP_INTERFACE_VAL="${PILOT_AP_INTERFACE:-wlan1}"
AP_SSID_VAL="${PILOT_AP_SSID:-}"
AP_PASSPHRASE_VAL="${PILOT_AP_PASSPHRASE:-}"
AP_IP_VAL="${PILOT_AP_IP:-192.168.50.1/24}"
AP_DHCP_RANGE_VAL="${PILOT_AP_DHCP_RANGE:-192.168.50.10,192.168.50.100}"
AP_DHCP_LEASE_VAL="${PILOT_AP_DHCP_LEASE:-12h}"
AP_MDNS_NAME_VAL="${PILOT_AP_MDNS_NAME:-}"
AP_DRY_RUN_VAL="${PILOT_AP_DRY_RUN:-false}"

# Export for any nodes that may read env
export PILOT_WEB_PORT="$WEB_PORT_VAL"
export PILOT_WS_PORT="$WEBSOCKET_PORT_VAL"
export PILOT_CMD_VEL_TOPIC="$CMD_VEL_TOPIC_VAL"
export PILOT_HOST="$HOST_VAL"
export PILOT_VOICE_TOPIC="$VOICE_TOPIC_VAL"
export PILOT_GPS_FIX_TOPIC="$GPS_FIX_TOPIC_VAL"
export PILOT_CONVERSATION_TOPIC="$CONV_TOPIC_VAL"
export PILOT_ENABLE_HTTP="$ENABLE_HTTP_VAL"
export PILOT_ENABLE_WS="$ENABLE_WS_VAL"
export PILOT_RUN_SEPARATE_WS="$RUN_SEPARATE_WS_VAL"
export PILOT_ENABLE_AP="$ENABLE_AP_VAL"
export PILOT_AP_INTERFACE="$AP_INTERFACE_VAL"
export PILOT_AP_SSID="$AP_SSID_VAL"
export PILOT_AP_PASSPHRASE="$AP_PASSPHRASE_VAL"
export PILOT_AP_IP="$AP_IP_VAL"
export PILOT_AP_DHCP_RANGE="$AP_DHCP_RANGE_VAL"
export PILOT_AP_DHCP_LEASE="$AP_DHCP_LEASE_VAL"
export PILOT_AP_MDNS_NAME="$AP_MDNS_NAME_VAL"
export PILOT_AP_DRY_RUN="$AP_DRY_RUN_VAL"

echo "[pilot/launch] Starting pilot web interface..."
echo "[pilot/launch] Web: http://${HOST_VAL}:${WEB_PORT_VAL}"
echo "[pilot/launch] WebSocket: ws://${HOST_VAL}:${WEBSOCKET_PORT_VAL}"
echo "[pilot/launch] Publishing to: ${CMD_VEL_TOPIC_VAL}"

ros2 launch pilot pilot.launch.py \
  web_port:="${WEB_PORT_VAL}" \
  websocket_port:="${WEBSOCKET_PORT_VAL}" \
  cmd_vel_topic:="${CMD_VEL_TOPIC_VAL}" \
  voice_topic:="${VOICE_TOPIC_VAL}" \
  gps_fix_topic:="${GPS_FIX_TOPIC_VAL}" \
  conversation_topic:="${CONV_TOPIC_VAL}" \
  host:="${HOST_VAL}" \
  enable_http:="${ENABLE_HTTP_VAL}" \
  enable_websocket:="${ENABLE_WS_VAL}" \
  run_separate_websocket:="${RUN_SEPARATE_WS_VAL}" \
  enable_ap:="${ENABLE_AP_VAL}" \
  ap_interface:="${AP_INTERFACE_VAL}" \
  ap_ssid:="${AP_SSID_VAL}" \
  ap_passphrase:="${AP_PASSPHRASE_VAL}" \
  ap_ip:="${AP_IP_VAL}" \
  dhcp_range:="${AP_DHCP_RANGE_VAL}" \
  dhcp_lease_time:="${AP_DHCP_LEASE_VAL}" \
  mdns_name:="${AP_MDNS_NAME_VAL}" \
  ap_dry_run:="${AP_DRY_RUN_VAL}" \
  ${@:-}
