#!/usr/bin/env bash
set -euo pipefail

REAL_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$REAL_PATH")"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
HOST_SHORT="${HOST:-$(hostname -s)}"

CONFIG_FILE=""
if [ -f "${REPO_DIR}/hosts/${HOST_SHORT}/config/will.env" ]; then
  CONFIG_FILE="${REPO_DIR}/hosts/${HOST_SHORT}/config/will.env"
elif [ -f "${REPO_DIR}/config/will.env" ]; then
  CONFIG_FILE="${REPO_DIR}/config/will.env"
fi

if [ -n "$CONFIG_FILE" ] && [ -f "$CONFIG_FILE" ]; then
  # shellcheck disable=SC1090
  source "$CONFIG_FILE"
fi

ros2 launch psyched_bt brain.launch.py "$@"
