#!/bin/bash
set -euo pipefail
# Stop Wi-Fi AP services
sudo pkill hostapd || true
sudo pkill dnsmasq || true
sudo pkill avahi-daemon || true
sudo pkill -f 'python3 -m http.server' || true
echo "[shutdown_wifi] Stopped Wi-Fi AP services."
