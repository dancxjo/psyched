#!/usr/bin/env bash
set -euo pipefail

# --------------------------------------------------------------------
# Pete Rizzlington bootstrap setup
# --------------------------------------------------------------------

# 1. Update package index and install core dependencies in a single transaction
psyched_bootstrap__resolve_script_dir() {
    local source="$1"

    while [[ -h "${source}" ]]; do
        local dir
        dir="$(cd -P "$(dirname "${source}")" && pwd)"
        source="$(readlink "${source}")"
        if [[ "${source}" != /* ]]; then
            source="${dir}/${source}"
        fi
    done

    (cd -P "$(dirname "${source}")" && pwd)
}

script_dir="$(psyched_bootstrap__resolve_script_dir "${BASH_SOURCE[0]}")"
# shellcheck source=tools/bootstrap/profile_helpers.sh
source "${script_dir}/profile_helpers.sh"
# shellcheck source=tools/bootstrap/reboot_helpers.sh
source "${script_dir}/reboot_helpers.sh"
unset -f psyched_bootstrap__resolve_script_dir

# Recompute using the shared helper so downstream scripts can reference it as
# well, keeping `script_dir` stable even when invoked through additional
# symlink layers.
script_dir="$(psyched_bootstrap::script_dir "${BASH_SOURCE[0]}")"

# Clear any legacy reboot sentinel so the new bootstrap flow can continue
# without forcing a system restart.
psyched_bootstrap::clear_reboot_sentinel
sudo apt-get update

CORE_PACKAGES=(
  ca-certificates
  build-essential
  curl
  git
  make
  python3
  python3-pip
  python3-venv
  python-is-python3
  shellcheck
  ripgrep
  unzip
  avahi-daemon
  avahi-utils
  libnss-mdns
)

sudo apt-get install -y --no-install-recommends "${CORE_PACKAGES[@]}"

# Ensure ~/.local/bin is on PATH for the current session and future logins
if [[ ":${PATH}:" != *":${HOME}/.local/bin:"* ]]; then
    export PATH="${HOME}/.local/bin:${PATH}"
fi
local_path_export="export PATH=\"\$HOME/.local/bin:\$PATH\""
psyched_bootstrap::append_profile_line "$HOME/.bashrc" "$local_path_export"

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

# 4. ROS tooling intentionally minimal; avoid installing python3-colcon-* to prevent catkin conflicts

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

echo
echo "Running 'psh host setup' to provision host prerequisites..."
(
    cd "${repo_root}" >/dev/null 2>&1
    psh host setup
)

echo
echo "Bootstrap and host provisioning complete."
echo "Open a new terminal (or source ~/.bashrc) before running 'psh mod setup' and 'psh svc setup' to finish provisioning."
echo "To rerun everything in one pass later, use 'psh host setup --include-modules --include-services'."
