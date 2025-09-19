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
sudo apt-get install -y --no-install-recommends git \
    build-essential curl make \
    python-is-python3 \
    python3-venv \
    python3-pip \
    python3-setuptools \
    python3-dev

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

        # Ensure automatic environment setup in user's .bashrc (idempotent)
        BASHRC="$HOME/.bashrc"
        BASHRC_MARK_START="# >>> psyched auto-setup >>>"
        BASHRC_MARK_END="# <<< psyched auto-setup <<<"
        AUTO_BLOCK=$(cat <<'BRC'
# >>> psyched auto-setup >>>
# Automatically set up ROS 2 and project venv for this repository in interactive shells.
# This is idempotent and safe if the repo directory moves; adjust PSYCHED_REPO if needed.
PSYCHED_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
if [ -d "$PSYCHED_REPO" ] && [ -f "$PSYCHED_REPO/tools/setup_env.sh" ]; then
    # Only run in interactive shells
    case $- in
        *i*)
            if command -v bash >/dev/null 2>&1; then
                eval "$(SETUP_ENV_MODE=print "$PSYCHED_REPO"/tools/setup_env.sh)"
            fi
            ;;
    esac
fi
# <<< psyched auto-setup <<<
BRC
)

        if ! grep -Fq "$BASHRC_MARK_START" "$BASHRC" 2>/dev/null; then
                echo "[bootstrap] Adding auto-setup block to $BASHRC"
                {
                    echo "$BASHRC_MARK_START"
                    echo "$AUTO_BLOCK"
                    echo "$BASHRC_MARK_END"
                } >> "$BASHRC"
        else
                echo "[bootstrap] Auto-setup block already present in $BASHRC; skipping"
        fi

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
