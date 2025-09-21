#!/usr/bin/env bash
set -euo pipefail

echo "[gps/teardown] DEPRECATED: use shutdown.sh instead. Forwarding to shutdown.sh if present..."
REAL_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$REAL_PATH")"
SHUTDOWN="$SCRIPT_DIR/shutdown.sh"
if [ -f "$SHUTDOWN" ]; then
  if [ -x "$SHUTDOWN" ]; then
    ( cd "$SCRIPT_DIR" && ./shutdown.sh )
  else
    ( cd "$SCRIPT_DIR" && bash ./shutdown.sh )
  fi
else
  echo "[gps/teardown] No shutdown.sh present; nothing to do."
fi

