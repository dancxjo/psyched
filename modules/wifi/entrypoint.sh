#!/bin/bash
set -euo pipefail

IFACE=${IFACE:-wlan0}
AP_IP=${AP_IP:-192.168.50.1}
HOSTNAME=$(hostname)

echo "[entrypoint] using hostname: $HOSTNAME.local"

# configure interface
ip link set $IFACE down || true
ip addr flush dev $IFACE || true
ip addr add $AP_IP/24 dev $IFACE
ip link set $IFACE up

# inject dynamic hostname into dnsmasq config
echo "address=/$HOSTNAME.local/$AP_IP" >> /etc/dnsmasq.conf

# inject hostname into avahi config
sed -i "s/^#*host-name=.*/host-name=$HOSTNAME/" /etc/avahi/avahi-daemon.conf

# start services
hostapd /etc/hostapd/hostapd.conf &
dnsmasq -C /etc/dnsmasq.conf &
avahi-daemon --no-chroot -f /etc/avahi/avahi-daemon.conf &
(cd /srv/www && python3 -m http.server 80) &
wait -n
