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
    alsa-utils ffmpeg \
    build-essential curl make \
    python-is-python3 \
    python3-full \
    python3-venv \
    python3-pip \
    python3-setuptools \
    python3-dev \
    piper

# Install Piper TTS globally for system services
echo "[bootstrap] Installing Piper TTS globally..."
if ! python3 -c 'import piper' >/dev/null 2>&1; then
    if sudo pip3 install piper-tts >/dev/null 2>&1; then
        echo "[bootstrap] Installed piper-tts globally"
    else
        echo "[bootstrap] Warning: Failed to install piper-tts globally. Modules will install locally."
    fi
else
    echo "[bootstrap] Piper TTS already installed"
fi

# Ensure Piper voices directory with proper permissions
echo "[bootstrap] Setting up Piper voices directory..."
PIPER_VOICES_DIR="${PIPER_VOICES_DIR:-/opt/piper/voices}"
if sudo mkdir -p "$PIPER_VOICES_DIR" >/dev/null 2>&1; then
    # Make the directory writable by the current user and group for downloads
    sudo chown "$USER:$USER" "$PIPER_VOICES_DIR" || true
    sudo chmod 755 "$PIPER_VOICES_DIR" || true
    echo "[bootstrap] Piper voices directory ready at $PIPER_VOICES_DIR"
else
    echo "[bootstrap] Warning: Could not create $PIPER_VOICES_DIR directory"
fi

# Ensure ROS2 is available
echo "[bootstrap] Ensuring ROS2 via 'make ros2'..."
make ros2

# Source repository environment (ROS + venv) so module setups use venv, not system Python
echo "[bootstrap] Sourcing repo environment (ROS + venv)..."
if ! eval "$(SETUP_ENV_MODE=print "$REPO_ROOT"/tools/setup_env.sh)"; then
    echo "[bootstrap] Error: failed to source repo environment." >&2
    exit 1
fi

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

            # Ensure automatic environment setup in user's .bashrc (replace if present)
        BASHRC="$HOME/.bashrc"
        BASHRC_MARK_START="# >>> psyched auto-setup >>>"
        BASHRC_MARK_END="# <<< psyched auto-setup <<<"
            AUTO_BLOCK=$(cat <<'BRC'
# >>> psyched auto-setup >>>
# Automatically set up ROS 2 and project venv for this repository in interactive shells.
PSYCHED_REPO="__PSYCHED_REPO_PLACEHOLDER__"
if [ -d "$PSYCHED_REPO" ] && [ -f "$PSYCHED_REPO/tools/setup_env.sh" ]; then
    case $- in
        *i*)
            eval "$(SETUP_ENV_MODE=print "$PSYCHED_REPO"/tools/setup_env.sh)"
            ;;
    esac
fi
# <<< psyched auto-setup <<<
BRC
)

            # Replace placeholder with actual path safely
            AUTO_BLOCK="${AUTO_BLOCK//__PSYCHED_REPO_PLACEHOLDER__/$REPO_ROOT}"

            if grep -Fq "$BASHRC_MARK_START" "$BASHRC" 2>/dev/null; then
                    echo "[bootstrap] Updating auto-setup block in $BASHRC"
                    awk -v start="$BASHRC_MARK_START" -v end="$BASHRC_MARK_END" 'BEGIN{skip=0} {
                        if ($0==start) {skip=1; next}
                        if ($0==end) {skip=0; next}
                        if (!skip) print
                    }' "$BASHRC" > "$BASHRC.tmp" && mv "$BASHRC.tmp" "$BASHRC"
            else
                    echo "[bootstrap] Adding auto-setup block to $BASHRC"
            fi
            {
                echo "$BASHRC_MARK_START"
                echo "$AUTO_BLOCK"
                echo "$BASHRC_MARK_END"
            } >> "$BASHRC"

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
