#!/usr/bin/env bash
# Strict mode: exit on error, treat unset vars as error, fail pipelines early
set -euo pipefail

# Remember starting directory and ensure we always return
STARTED_DIR="$(pwd)"
cleanup() { cd "$STARTED_DIR" || true; }
trap cleanup EXIT

# Resolve repository root (robust against symlinks and invocation dirs)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
cd "$REPO_ROOT"
echo "[bootstrap] Repository root: $REPO_ROOT"

# Ensure required system packages (Debian/Ubuntu)
echo "[bootstrap] Installing required system packages..."
sudo apt-get update -y
sudo apt-get install -y --no-install-recommends git build-essential curl make

# Ensure ROS2 is available
echo "[bootstrap] Ensuring ROS2 via 'make ros2'..."
make ros2

# Determine host (env var HOST overrides system hostname)
HOST="${HOST:-$(hostname -s)}"
HOST_DIR="$REPO_ROOT/hosts/$HOST"
ENABLED_DIR="$HOST_DIR/enabled"
echo "[bootstrap] Host: $HOST"

if [[ -d "$HOST_DIR" ]]; then
    # Source all enabled modules (skip if none)
    shopt -s nullglob
    for module in "$ENABLED_DIR"/*; do
        if [[ -f "$module/setup.sh" ]]; then
            echo "[bootstrap] Running host module setup: $module/setup.sh"
            # shellcheck source=/dev/null
            source "$module/setup.sh"
        fi
    done
    shopt -u nullglob

    echo "[bootstrap] Building workspace..."
    make build

    # Add launch script to crontab (idempotent)
    LAUNCH_SCRIPT="$REPO_ROOT/tools/launch/launch.sh"
    if [[ -x "$LAUNCH_SCRIPT" || -f "$LAUNCH_SCRIPT" ]]; then
        CRON_ENTRY="@reboot sleep 30 && $LAUNCH_SCRIPT &"
        if ! crontab -l 2>/dev/null | grep -Fqx "$CRON_ENTRY"; then
            echo "[bootstrap] Adding launch script to crontab (@reboot)"
            (crontab -l 2>/dev/null; echo "$CRON_ENTRY") | crontab -
        else
            echo "[bootstrap] Crontab entry already present; skipping"
        fi
    else
        echo "[bootstrap] Warning: launch script not found at $LAUNCH_SCRIPT; skipping crontab entry" >&2
    fi
else
    echo "[bootstrap] Error: no host configuration for '$HOST' at $HOST_DIR" >&2
    exit 1
fi
