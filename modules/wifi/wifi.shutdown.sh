#!/bin/bash
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  exec sudo -E "$0" "$@"
fi

REPO_DIR="${REPO_DIR:-$(cd "$(dirname "$0")/../.." && pwd)}"
MODULE_DIR="${MODULE_DIR:-$(cd "$(dirname "$0")" && pwd)}"
export REPO_DIR MODULE_DIR

IFACE="${WIFI_INTERFACE:-wlan0}"
STATE_DIR="${WIFI_STATE_DIR:-/run/psyched-wifi/${IFACE}}"
IFACE_STATE_FILE="${STATE_DIR}/iface_state.json"
NAT_STATE_FILE="${STATE_DIR}/nat_rules.json"

# Attempt graceful termination first
pkill -f 'ros2 run wifi wifi_ap_manager' >/dev/null 2>&1 || true
pkill hostapd >/dev/null 2>&1 || true
pkill dnsmasq >/dev/null 2>&1 || true

if [[ -f "$NAT_STATE_FILE" ]]; then
  python3 - "$NAT_STATE_FILE" <<'PY'
import json
import subprocess
import sys
from pathlib import Path

path = sys.argv[1]
try:
    data = json.loads(Path(path).read_text())
except Exception:
    data = {}

if isinstance(data, dict):
    rules = data.get('rules', [])
    ip_forward = data.get('ip_forward')
else:
    rules = data
    ip_forward = None

for rule in reversed(rules or []):
    rule = list(rule)
    if '-A' in rule:
        rule[rule.index('-A')] = '-D'
    subprocess.run(rule, check=False)

if ip_forward is not None:
    subprocess.run(['sysctl', f'net.ipv4.ip_forward={ip_forward}'], check=False)
else:
    subprocess.run(['sysctl', 'net.ipv4.ip_forward=0'], check=False)
PY
fi

if [[ -f "$IFACE_STATE_FILE" ]]; then
  python3 - "$IFACE" "$IFACE_STATE_FILE" <<'PY'
import json
import subprocess
import sys
from pathlib import Path

iface = sys.argv[1]
state_path = Path(sys.argv[2])
try:
    data = json.loads(state_path.read_text())
except Exception:
    data = []

subprocess.run(['ip', 'addr', 'flush', 'dev', iface], check=False)
oper_up = False
for entry in data:
    if entry.get('operstate') == 'UP':
        oper_up = True
    for info in entry.get('addr_info', []):
        if info.get('family') != 'inet':
            continue
        addr = info.get('local')
        prefix = info.get('prefixlen')
        if addr and prefix is not None:
            subprocess.run(['ip', 'addr', 'add', f'{addr}/{prefix}', 'dev', iface], check=False)
subprocess.run(['ip', 'link', 'set', iface, 'up' if oper_up else 'down'], check=False)
PY
else
  ip addr flush dev "$IFACE" || true
  ip link set "$IFACE" down || true
fi

echo "[shutdown_wifi] Wi-Fi AP services stopped and network state restored."
