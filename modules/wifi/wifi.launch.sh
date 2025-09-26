#!/bin/bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
IFACE=${IFACE:-wlan0}
AP_IP=${AP_IP:-192.168.50.1}
HOSTNAME=$(hostname)

sudo ip link set "$IFACE" down || true
sudo ip addr flush dev "$IFACE" || true
sudo ip addr add "$AP_IP/24" dev "$IFACE"
sudo ip link set "$IFACE" up

sudo cp dnsmasq.conf /etc/dnsmasq.conf
sudo bash -c "echo 'address=/$HOSTNAME.local/$AP_IP' >> /etc/dnsmasq.conf"

sudo cp avahi-daemon.conf /etc/avahi/avahi-daemon.conf
sudo sed -i "s/^#*host-name=.*/host-name=$HOSTNAME/" /etc/avahi/avahi-daemon.conf

sudo hostapd /etc/hostapd/hostapd.conf &
sudo dnsmasq -C /etc/dnsmasq.conf &
sudo avahi-daemon --no-chroot -f /etc/avahi/avahi-daemon.conf &
(cd www && python3 -m http.server 80) &
wait -n

REPO_DIR="${REPO_DIR:-$(cd "$(dirname "$0")/../.." && pwd)}"
export REPO_DIR
