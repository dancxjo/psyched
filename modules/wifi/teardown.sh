#!/bin/bash
set -euo pipefail
# Remove Wi-Fi AP configs (optional)
sudo rm -f /etc/hostapd/hostapd.conf /etc/dnsmasq.conf /etc/avahi/avahi-daemon.conf
echo "[teardown_wifi] Removed Wi-Fi AP configs."
