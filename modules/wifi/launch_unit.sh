#!/usr/bin/env bash
set -euo pipefail

if [[ ${EUID} -ne 0 ]]; then
  exec sudo -E "$0" "$@"
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="${REPO_DIR:-$(cd "${SCRIPT_DIR}/../.." && pwd)}"
HOST_SHORT="${HOST:-$(hostname -s)}"

PARAM_FILE="${WIFI_PARAMS_FILE:-}"
if [[ -z "${PARAM_FILE}" ]]; then
  LEGACY_YAML="${REPO_DIR}/hosts/${HOST_SHORT}/config/wifi.yaml"
  if [[ -f "${LEGACY_YAML}" ]]; then
    PARAM_FILE="${LEGACY_YAML}"
  fi
fi

if [[ -n "${PARAM_FILE}" ]]; then
  exec ros2 run wifi wifi_ap_manager --ros-args --params-file "${PARAM_FILE}"
fi

HOST_ARGS=()
CONFIG_FILE="${WIFI_CONFIG_FILE:-}"
if [[ -z "${CONFIG_FILE}" ]]; then
  for candidate in \
    "${REPO_DIR}/hosts/${HOST_SHORT}.json" \
    "${REPO_DIR}/hosts/${HOST_SHORT}.jsonc" \
    "${REPO_DIR}/hosts/${HOST_SHORT}.yaml" \
    "${REPO_DIR}/hosts/${HOST_SHORT}.yml" \
    "${REPO_DIR}/hosts/${HOST_SHORT}.toml"
  do
    if [[ -f "${candidate}" ]]; then
      CONFIG_FILE="${candidate}"
      break
    fi
  done
fi

if [[ -n "${CONFIG_FILE}" ]]; then
  mapfile -t HOST_ARGS < <(python3 "${REPO_DIR}/tools/launch_args.py" --module wifi "${CONFIG_FILE}" || true)
fi

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
  -p "interface:=${IFACE}"
  -p "ssid:=${SSID}"
  -p "ap_ip:=${AP_IP}"
  -p "dhcp_range:=${DHCP_RANGE}"
  -p "dhcp_lease_time:=${DHCP_LEASE}"
  -p "channel:=${CHANNEL}"
  -p "country_code:=${COUNTRY}"
  -p "mdns_name:=${MDNS_NAME}"
  -p "http_port:=${HTTP_PORT}"
  -p "websocket_port:=${WS_PORT}"
  -p "state_dir:=${STATE_DIR}"
)

if [[ -n "${PASSPHRASE}" ]]; then
  ROS_ARGS+=(-p "passphrase:=${PASSPHRASE}")
fi

if [[ "${ENABLE_NAT,,}" == "true" ]]; then
  ROS_ARGS+=(-p "enable_nat:=true" -p "internet_interface:=${INTERNET_IFACE}")
else
  ROS_ARGS+=(-p "enable_nat:=false")
fi

if [[ ${#HOST_ARGS[@]} -gt 0 ]]; then
  for param in "${HOST_ARGS[@]}"; do
    ROS_ARGS+=(-p "${param}")
  done
fi

echo "[wifi/launch] Starting Wi-Fi AP on ${IFACE} (SSID: ${SSID})"
if [[ "${ENABLE_NAT,,}" == "true" ]]; then
  echo "[wifi/launch] Internet sharing enabled via ${INTERNET_IFACE}"
fi

exec ros2 run wifi wifi_ap_manager "${ROS_ARGS[@]}"
