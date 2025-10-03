#!/usr/bin/env bash
set -euo pipefail

# --------------------------------------------------------------------
# Pete Rizzlington bootstrap setup
# --------------------------------------------------------------------

# 1. Update & upgrade
sudo apt update && sudo apt upgrade -y

# 2. Essentials: compiler, Python (for ROS tooling), editors
sudo apt install -y \
  curl git build-essential make \
  python3-full python3-pip python3-venv python-is-python3 \
  shellcheck micro mc ripgrep

# Ensure ~/.local/bin is on PATH for the current session and future logins
if [[ ":${PATH}:" != *":${HOME}/.local/bin:"* ]]; then
    export PATH="${HOME}/.local/bin:${PATH}"
fi
local_path_export="export PATH=\"\$HOME/.local/bin:\$PATH\""
if ! grep -Fx "$local_path_export" "$HOME/.bashrc" >/dev/null 2>&1; then
    printf '%s
' "$local_path_export" >> "$HOME/.bashrc"
fi

# 3. mDNS support (Avahi)
sudo apt install -y avahi-daemon avahi-utils libnss-mdns

# Ensure /etc/nsswitch.conf has mdns entries
ensure_mdns_hosts_entry() {
    local NSSWITCH="/etc/nsswitch.conf"
    if [ ! -f "${NSSWITCH}" ]; then
        echo "Warning: ${NSSWITCH} not found; skipping mDNS resolver configuration" >&2
        return 0
    fi
    if grep -E '^[[:space:]]*hosts:.*mdns' "${NSSWITCH}" >/dev/null 2>&1; then
        return 0
    fi
    echo "Adding mDNS resolution to ${NSSWITCH}..." >&2
    sudo python3 - "$NSSWITCH" <<'PY'
import shutil, sys
from pathlib import Path

path = Path(sys.argv[1])
backup = path.with_name(path.name + ".bak")
if not backup.exists():
    shutil.copy2(path, backup)

lines = path.read_text().splitlines()
preferred = ["files", "mdns4_minimal", "[NOTFOUND=return]", "dns", "mdns"]

for idx, line in enumerate(lines):
    if not line.startswith("hosts:"):
        continue
    prefix, sep, remainder = line.partition(":")
    tokens = remainder.split()
    for token in preferred:
        if token not in tokens:
            if token == "dns" and "mdns" in tokens:
                tokens.insert(tokens.index("mdns"), token)
            else:
                tokens.append(token)
    lines[idx] = f"{prefix}{sep}\t{' '.join(tokens)}"
    break
else:
    lines.append("hosts:\t" + " ".join(preferred))

path.write_text("\n".join(lines) + "\n")
PY
}

# Ensure avahi-daemon enabled
ensure_mdns_service() {
    if ! command -v systemctl >/dev/null 2>&1; then
        echo "systemctl not available; skipping Avahi enablement." >&2
        return 0
    fi
    if ! systemctl list-unit-files | grep -q '^avahi-daemon\.service'; then
        echo "avahi-daemon.service not present; skipping enablement." >&2
        return 0
    fi
    echo "Ensuring avahi-daemon.service is enabled and running..." >&2
    sudo systemctl unmask avahi-daemon.service >/dev/null 2>&1 || true
    sudo systemctl enable avahi-daemon.service >/dev/null 2>&1 || true
    sudo systemctl restart avahi-daemon.service >/dev/null 2>&1 || sudo systemctl start avahi-daemon.service || true
}

ensure_mdns_hosts_entry
ensure_mdns_service

# 4. Convenience tools
if ! sudo apt install -y unzip 1>&2; then
    echo "Warning: failed to install unzip; continuing" >&2
fi

# 5. ROS colcon tooling
sudo apt install -y python3-colcon-*

install_colcon_ros_cargo() {
    if python3 -m pip show colcon-ros-cargo >/dev/null 2>&1; then
        local version
        version=$(python3 -m pip show colcon-ros-cargo | awk '/^Version:/ {print $2}')
        echo "colcon-ros-cargo already installed (version ${version:-unknown})"
        return 0
    fi

    echo "Installing colcon-ros-cargo from GitHub..."
    python3 -m pip install --user --upgrade --break-system-packages pip 
    python3 -m pip install --user --upgrade --break-system-packages \
        "git+https://github.com/colcon/colcon-ros-cargo.git"
}

install_colcon_ros_cargo

# 6. Rust toolchain (rustup + cargo)
if ! command -v cargo >/dev/null 2>&1; then
    echo "Installing Rust toolchain via rustup..."
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
    export PATH="$HOME/.cargo/bin:$PATH"
else
    echo "Rust already installed: $(cargo --version)"
fi

# 7. Build psh
echo "Building psh crate..."
cd "$(dirname "$0")/psh"
cargo build --release --package psh
cd -

# Install globally as /usr/bin/psh (symlink to the freshly built binary)
repo_root="$(cd "$(dirname "$0")" && pwd)"
psh_binary="${repo_root}/psh/target/debug/psh"

if [ ! -x "${psh_binary}" ]; then
    echo "Error: expected psh binary at ${psh_binary} (did cargo build --release succeed?)" >&2
    exit 1
fi

echo "Installing psh binary -> /usr/bin/psh"
sudo ln -sf "${psh_binary}" /usr/bin/psh
sudo chmod a+x /usr/bin/psh

# 8. Run psh setup
echo "Launching psh setup..."
exec /usr/bin/psh setup
