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
    python3-pip \
    python3-setuptools \
    python3-dev

# Prefer system-wide packages; Python modules will be installed globally (no venv)
echo "[bootstrap] Using system-wide Python (no virtualenv)"

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

# Source repository environment (ROS + workspace) for builds and module setups
echo "[bootstrap] Sourcing repo environment (ROS + workspace)..."
if [ -f "$REPO_ROOT/tools/setup_env.sh" ]; then
    # shellcheck disable=SC1090
    source "$REPO_ROOT/tools/setup_env.sh"
else
    echo "[bootstrap] Warning: tools/setup_env.sh not found; continuing without sourcing" >&2
fi

# Ensure piper-tts is available system-wide
echo "[bootstrap] Ensuring piper-tts is installed (system-wide)..."
if ! python3 -c 'import piper' >/dev/null 2>&1; then
    # Upgrade pip quietly if allowed
    pip3 install --break-system-packages --no-input --upgrade pip >/dev/null 2>&1 || true
    if pip3 install --break-system-packages piper-tts >/dev/null 2>&1; then
        echo "[bootstrap] Installed piper-tts via pip3"
    else
        echo "[bootstrap] Warning: Failed to install piper-tts via pip3 (may require sudo). Trying sudo..." >&2
        if command -v sudo >/dev/null 2>&1; then
            if sudo pip3 install --break-system-packages piper-tts >/dev/null 2>&1; then
                echo "[bootstrap] Installed piper-tts via sudo pip3"
            else
                echo "[bootstrap] Warning: sudo pip3 install of piper-tts failed. Voice module may fallback to a binary if available." >&2
            fi
        fi
    fi
else
    echo "[bootstrap] piper-tts already present"
fi

# Determine host (env var HOST overrides system hostname)
HOST="${HOST:-$(hostname -s)}"
HOST_DIR="$REPO_ROOT/hosts/$HOST"
ENABLED_DIR="$HOST_DIR/modules"
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
# Automatically set up ROS 2 and this workspace environment in interactive shells.
PSYCHED_REPO="__PSYCHED_REPO_PLACEHOLDER__"
if [ -d "$PSYCHED_REPO" ] && [ -f "$PSYCHED_REPO/tools/setup_env.sh" ]; then
    case $- in
        *i*)
            eval "$(SETUP_ENV_MODE=print bash \"$PSYCHED_REPO\"/tools/setup_env.sh)"
            ;;
    esac
fi
BRC
)

            # Replace placeholder with actual path safely
            AUTO_BLOCK="${AUTO_BLOCK//__PSYCHED_REPO_PLACEHOLDER__/$REPO_ROOT}"

            echo "[bootstrap] Ensuring auto-setup block in $BASHRC"
            # Ensure bashrc exists
            touch "$BASHRC"
            # Remove any existing psyched auto-setup blocks (all occurrences)
            awk -v start="$BASHRC_MARK_START" -v end="$BASHRC_MARK_END" 'BEGIN{skip=0} {
                if ($0==start) {skip=1; next}
                if ($0==end) {skip=0; next}
                if (!skip) print
            }' "$BASHRC" > "$BASHRC.tmp" && mv "$BASHRC.tmp" "$BASHRC"
            # Append a single, properly marked block
            {
                echo "$BASHRC_MARK_START"
                echo "$AUTO_BLOCK"
                echo "$BASHRC_MARK_END"
            } >> "$BASHRC"

    # Add launch script to crontab (idempotent)
    LAUNCH_SCRIPT="$REPO_ROOT/tools/launch.sh"
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
