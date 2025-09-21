#!/usr/bin/env bash
set -euo pipefail
# Usage: systemd_entrypoint.sh <script> [args...]
# Sources ROS 2 and workspace env, then execs the given script with args
REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"
SETUP_SH="$REPO_DIR/tools/setup_env.sh"
if [ -f "$SETUP_SH" ]; then
  # Temporarily disable nounset while sourcing external setup to avoid failures
  # from upstream scripts that assume unset variables.
  had_u=0
  case "$-" in *u*) had_u=1 ;; esac
  if (( had_u )); then set +u; fi
  # shellcheck disable=SC1091
  source "$SETUP_SH" || {
    printf "# [systemd_entrypoint] Failed to source %s\n" "$SETUP_SH" >&2
  }
  if (( had_u )); then set -u; fi
  printf "# [systemd_entrypoint] sourced: %s\n" "$SETUP_SH" >&2
fi
exec "$@"
