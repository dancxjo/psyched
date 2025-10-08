#!/usr/bin/env bash
set -euo pipefail
if command -v systemctl >/dev/null 2>&1; then
  sudo systemctl enable gpsd.service || true
  sudo systemctl restart gpsd.service || sudo systemctl start gpsd.service || true
fi
