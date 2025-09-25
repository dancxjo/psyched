
#!/bin/bash
set -euo pipefail
IFACE=${IFACE:-wlan0}
AP_IP=${AP_IP:-192.168.50.1}
HOSTNAME=$(hostname)

# configure interface
sudo ip link set $IFACE down || true
sudo ip addr flush dev $IFACE || true
sudo ip addr add $AP_IP/24 dev $IFACE
sudo ip link set $IFACE up

# inject dynamic hostname into dnsmasq config
sudo cp dnsmasq.conf /etc/dnsmasq.conf
sudo bash -c "echo 'address=/$HOSTNAME.local/$AP_IP' >> /etc/dnsmasq.conf"

# inject hostname into avahi config
sudo cp avahi-daemon.conf /etc/avahi/avahi-daemon.conf
sudo sed -i "s/^#*host-name=.*/host-name=$HOSTNAME/" /etc/avahi/avahi-daemon.conf

# start services
sudo hostapd /etc/hostapd/hostapd.conf &
sudo dnsmasq -C /etc/dnsmasq.conf &
sudo avahi-daemon --no-chroot -f /etc/avahi/avahi-daemon.conf &
(cd www && python3 -m http.server 80) &
wait -n
