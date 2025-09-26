
#!/bin/bash
set -euo pipefail

# Always resolve REPO_DIR and MODULE_DIR
REPO_DIR="${REPO_DIR:-$(cd "$(dirname "$0")/../.." && pwd)}"
MODULE_NAME="pilot"
MODULE_DIR="$REPO_DIR/modules/$MODULE_NAME"
export REPO_DIR
export MODULE_DIR
cd "$MODULE_DIR"

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

echo "[pilot/launch] Starting pilot web interface..."
echo "[pilot/launch] Web: http://${HOST_VAL}:${WEB_PORT_VAL}"
echo "[pilot/launch] WebSocket: ws://${HOST_VAL}:${WEBSOCKET_PORT_VAL}"
echo "[pilot/launch] Publishing to: ${CMD_VEL_TOPIC_VAL}"

exec ros2 launch pilot pilot.launch.py \
  web_port:=${PILOT_WEB_PORT:-8080} \
  websocket_port:=${PILOT_WS_PORT:-8081} \
  cmd_vel_topic:=${PILOT_CMD_VEL_TOPIC:-/cmd_vel} \
  voice_topic:=${PILOT_VOICE_TOPIC:-/voice} \
  imu_topic:=${PILOT_IMU_TOPIC:-/imu} \
  gps_fix_topic:=${PILOT_GPS_FIX_TOPIC:-/gps/fix} \
  conversation_topic:=${PILOT_CONVERSATION_TOPIC:-/conversation} \
  host:=${PILOT_HOST:-0.0.0.0} \
  enable_http:=${PILOT_ENABLE_HTTP:-true} \
  enable_websocket:=${PILOT_ENABLE_WS:-false} \
  run_separate_websocket:=${PILOT_RUN_SEPARATE_WS:-true}
