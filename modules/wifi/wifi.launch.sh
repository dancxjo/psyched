#!/bin/bash
set -euo pipefail
REPO_DIR="${REPO_DIR:-$(cd "$(dirname "$0")/../.." && pwd)}"
export REPO_DIR
HOST_SHORT="${HOST:-$(hostname -s)}"

HOST_YAML="${REPO_DIR}/hosts/${HOST_SHORT}/config/wifi.yaml"

if [ -f "$HOST_YAML" ]; then
  exec ros2 launch wifi wifi.launch.py --ros-args --params-file "$HOST_YAML"
else
  exec ros2 launch wifi wifi.launch.py
fi
#!/bin/bash
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  exec sudo -E "$0" "$@"
fi

REPO_DIR="${REPO_DIR:-$(cd "$(dirname "$0")/../.." && pwd)}"
MODULE_DIR="${MODULE_DIR:-$(cd "$(dirname "$0")" && pwd)}"
export REPO_DIR MODULE_DIR

HOSTNAME_SHORT="$(hostname -s)"

IFACE="${WIFI_INTERFACE:-wlan0}"
SSID="${WIFI_SSID:-psyched-${HOSTNAME_SHORT}}"
PASSPHRASE="${WIFI_PASSPHRASE:-}"
AP_IP="${WIFI_AP_IP:-192.168.50.1/24}"
DHCP_RANGE="${WIFI_DHCP_RANGE:-192.168.50.10,192.168.50.100}"
DHCP_LEASE="${WIFI_DHCP_LEASE:-12h}"
CHANNEL="${WIFI_CHANNEL:-6}"
COUNTRY="${WIFI_COUNTRY_CODE:-US}"
MDNS_NAME="${WIFI_MDNS_NAME:-${HOSTNAME_SHORT}}"
HTTP_PORT="${WIFI_HTTP_PORT:-8080}"
WS_PORT="${WIFI_WS_PORT:-8081}"
ENABLE_NAT="${WIFI_ENABLE_NAT:-false}"
INTERNET_IFACE="${WIFI_UPLINK_INTERFACE:-eth0}"
STATE_DIR="${WIFI_STATE_DIR:-/run/psyched-wifi/${IFACE}}"

ROS_ARGS=(
  --ros-args
  -p interface:="$IFACE"
  -p ssid:="$SSID"
  -p ap_ip:="$AP_IP"
  -p dhcp_range:="$DHCP_RANGE"
  -p dhcp_lease_time:="$DHCP_LEASE"
  -p channel:=$CHANNEL
  -p country_code:="$COUNTRY"
  -p mdns_name:="$MDNS_NAME"
  -p http_port:=$HTTP_PORT
  -p websocket_port:=$WS_PORT
  -p state_dir:="$STATE_DIR"
)

if [[ -n "$PASSPHRASE" ]]; then
  ROS_ARGS+=(-p passphrase:="$PASSPHRASE")
fi

if [[ "${ENABLE_NAT,,}" == "true" ]]; then
  ROS_ARGS+=(-p enable_nat:=true -p internet_interface:="$INTERNET_IFACE")
else
  ROS_ARGS+=(-p enable_nat:=false)
fi

echo "[wifi/launch] Starting Wi-Fi AP on ${IFACE} (SSID: ${SSID})"
if [[ "${ENABLE_NAT,,}" == "true" ]]; then
  echo "[wifi/launch] Internet sharing enabled via ${INTERNET_IFACE}"
fi

exec "$REPO_DIR/tools/with_ros_env.sh" ros2 run wifi wifi_ap_manager "${ROS_ARGS[@]}"
