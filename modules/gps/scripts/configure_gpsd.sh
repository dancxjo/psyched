#!/usr/bin/env bash
set -euo pipefail
if [ -f /etc/default/gpsd ]; then
  sudo sed -i 's/^START_DAEMON=.*/START_DAEMON="true"/' /etc/default/gpsd || true
  sudo sed -i 's#^USBAUTO=.*#USBAUTO="true"#' /etc/default/gpsd || true
  if grep -q '^DEVICES=' /etc/default/gpsd; then
    sudo sed -i 's#^DEVICES=.*#DEVICES="/dev/gps0"#' /etc/default/gpsd || true
  else
    echo 'DEVICES="/dev/gps0"' | sudo tee -a /etc/default/gpsd >/dev/null
  fi
  if grep -q '^GPSD_OPTIONS=' /etc/default/gpsd; then
    sudo sed -i 's#^GPSD_OPTIONS=.*#GPSD_OPTIONS="-n"#' /etc/default/gpsd || true
  else
    echo 'GPSD_OPTIONS="-n"' | sudo tee -a /etc/default/gpsd >/dev/null
  fi
fi
