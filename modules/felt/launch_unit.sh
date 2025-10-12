#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="${REPO_DIR:-$(cd "${SCRIPT_DIR}/.." && pwd)}"
export REPO_DIR

ARGS=()

if [[ -n "${FELT_DEBOUNCE_SECONDS:-}" ]]; then
  ARGS+=("debounce_seconds:=${FELT_DEBOUNCE_SECONDS}")
fi
if [[ -n "${FELT_WINDOW_SECONDS:-}" ]]; then
  ARGS+=("window_seconds:=${FELT_WINDOW_SECONDS}")
fi

exec ros2 launch felt felt.launch.py "${ARGS[@]}"
