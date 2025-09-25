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
echo "[bootstrap] Installing required baseline system packages..."
sudo apt-get update -y
sudo apt-get install -y --no-install-recommends git \
    build-essential curl make \
    python-is-python3 \
    python3-full \
    python3-pip \
    python3-setuptools \
    python3-dev

# Prefer system-wide packages; Python modules will be installed globally (no venv)
echo "[bootstrap] Using system-wide Python (no virtualenv)"

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

# Module-specific dependencies (e.g., audio, TTS) are handled by module actions declared in module.toml

# Determine host (env var HOST overrides system hostname)
HOST="${HOST:-$(hostname -s)}"
HOST_DIR="$REPO_ROOT/hosts/$HOST"
echo "[bootstrap] Host: $HOST"

if [[ -d "$HOST_DIR" ]]; then
    echo "[bootstrap] Running host module setups via tools/setup..."
    "$REPO_ROOT/tools/setup"

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


else
    echo "[bootstrap] Error: no host configuration for '$HOST' at $HOST_DIR" >&2
    exit 1
fi
