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
    printf '%s\n' "$local_path_export" >> "$HOME/.bashrc"
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

# 6. Deno runtime
install_deno() {
    if command -v deno >/dev/null 2>&1; then
        echo "deno already installed: $(deno --version | head -n 1)"
        return 0
    fi

    echo "Installing deno runtime..."
    curl -fsSL https://deno.land/install.sh | sh -s -- -y
    export DENO_INSTALL="${HOME}/.deno"
    export PATH="${DENO_INSTALL}/bin:${PATH}"

    deno_bin="${DENO_INSTALL}/bin/deno"
    if [ ! -x "${deno_bin}" ]; then
        echo "Error: deno installer did not produce ${deno_bin}" >&2
        exit 1
    fi

    if ! command -v deno >/dev/null 2>&1; then
        echo "Warning: deno not on PATH after install; adding to shell profile" >&2
    fi

    deno_export="export PATH=\"${DENO_INSTALL}/bin:\$PATH\""
    if ! grep -Fx "${deno_export}" "${HOME}/.bashrc" >/dev/null 2>&1; then
        printf '%s\n' "${deno_export}" >> "${HOME}/.bashrc"
    fi
}

install_deno

deno_path="$(command -v deno || true)"
if [ -z "${deno_path}" ]; then
    echo "Error: deno not found on PATH after installation." >&2
    exit 1
fi

# 7. Install psh wrapper that executes the Deno CLI
repo_root="$(cd "$(dirname "$0")" && pwd)"
psh_wrapper="/usr/local/bin/psh"

tmp_wrapper="$(mktemp)"
cat > "${tmp_wrapper}" <<'PSH'
#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="__REPO_ROOT__"
export PSYCHED_REPO_ROOT="${REPO_ROOT}"

exec "__DENO_BIN__" run -A --config "${REPO_ROOT}/tools/psh/deno.json" "${REPO_ROOT}/tools/psh/main.ts" "$@"
PSH

sed -i "s#__REPO_ROOT__#${repo_root//\/\\/}#g" "${tmp_wrapper}"
sed -i "s#__DENO_BIN__#${deno_path//\/\\/}#g" "${tmp_wrapper}"
sudo install -m 0755 "${tmp_wrapper}" "${psh_wrapper}"
rm -f "${tmp_wrapper}"

if ! "${deno_path}" cache --config "${repo_root}/tools/psh/deno.json" "${repo_root}/tools/psh/main.ts" >/dev/null 2>&1; then
    echo "Warning: deno cache failed; dependencies will download on first run." >&2
fi

ensure_psyched_shell() {
    local env_script="${repo_root}/env/psyched_env.sh"
    local bashrc="${HOME}/.bashrc"
    local marker="# >>> psyched workspace >>>"
    local footer="# <<< psyched workspace <<<"

    if [[ ! -f "${env_script}" ]]; then
        return 0
    fi

    mkdir -p "${HOME}"
    touch "${bashrc}"

    if grep -Fq "${marker}" "${bashrc}" >/dev/null 2>&1; then
        return 0
    fi

    {
        printf '\n%s\n' "${marker}"
        printf '%s\n' "if [ -f \"${env_script}\" ]; then"
        printf '%s\n' '    # shellcheck disable=SC1091'
        printf '%s\n' "    source \"${env_script}\""
        printf '%s\n' '    psyched() {'
        printf '%s\n' '        if declare -F psyched::activate >/dev/null 2>&1; then'
        printf '%s\n' '            psyched::activate "$@"'
        printf '%s\n' '        else'
        printf '%s\n' '            echo "psyched::activate helper missing" >&2'
        printf '%s\n' '            return 1'
        printf '%s\n' '        fi'
        printf '%s\n' '    }'
        printf '%s\n' '    if [[ "${PSYCHED_AUTO_ACTIVATE:-1}" != "0" ]]; then'
        printf '%s\n' '        psyched --quiet'
        printf '%s\n' '    fi'
        printf '%s\n' 'fi'
        printf '%s\n' "${footer}"
    } >> "${bashrc}"
}

ensure_psyched_shell

# 8. Run the provisioning wizard
echo "Launching psh wizard..."
exec "${psh_wrapper}"
