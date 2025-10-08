#!/usr/bin/env bash
set -euo pipefail
if [ "${GPS_SKIP_UDEV:-0}" != "1" ]; then
  sudo install -d -m 0755 /etc/udev/rules.d
  cat <<'EOF' | sudo tee /etc/udev/rules.d/99-gps-ublox.rules >/dev/null
SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a7", SYMLINK+="gps0"
EOF
  sudo udevadm control --reload-rules
  sudo udevadm trigger || true
fi
