#!/bin/bash
set -euo pipefail
# Install Wi-Fi AP dependencies directly on host
sudo apt-get update
sudo apt-get install -y hostapd dnsmasq avahi-daemon iproute2 python3
# Enable and start services (will be configured by launch script)
echo "[install_wifi] Wi-Fi AP dependencies installed."
