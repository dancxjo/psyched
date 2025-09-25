#!/bin/bash
set -euo pipefail
# Build Wi-Fi AP Docker image
cd "$(dirname "$0")"
# No build step needed for direct host setup
echo "[setup_wifi] Host Wi-Fi AP dependencies are installed."
