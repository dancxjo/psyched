#!/usr/bin/env bash
set -euo pipefail
# Usage: systemd_entrypoint.sh <script> [args...]
# Sources ROS 2 and workspace env, then execs the given script with args
REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"
if [ -f "$REPO_DIR/tools/setup_env.sh" ]; then
  # shellcheck disable=SC1091
  source "$REPO_DIR/tools/setup_env.sh"
fi
exec "$@"
