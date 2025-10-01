#!/usr/bin/env bash
#
# install_docker.sh - Install Docker Engine and Docker Compose (plugin) on Debian/Ubuntu
# Installs Docker from the official Docker apt repository and enables the service.
# Usage: ./install_docker.sh

set -euo pipefail

SUDO=(sudo)
if [[ $(id -u) -eq 0 ]]; then
  SUDO=()
elif ! command -v sudo >/dev/null 2>&1; then
  echo "This script requires root privileges or sudo to escalate." >&2
  exit 1
fi

echo "Provisioning Docker Engine and Docker Compose (plugin) from Docker's official repository..."

export LANG=en_US.UTF-8

${SUDO[@]} apt update
${SUDO[@]} apt install -y ca-certificates curl gnupg lsb-release

echo "Setting up Docker apt repository and GPG key..."

# Create keyrings dir
${SUDO[@]} mkdir -p /etc/apt/keyrings

curl -fsSL https://download.docker.com/linux/ubuntu/gpg | ${SUDO[@]} gpg --dearmor -o /etc/apt/keyrings/docker.gpg
${SUDO[@]} chmod a+r /etc/apt/keyrings/docker.gpg

# Determine codename (fall back to 'focal' if unavailable)
OS_CODENAME=$(. /etc/os-release && echo "$VERSION_CODENAME")
if [[ -z "$OS_CODENAME" ]]; then
  OS_CODENAME=focal
fi

echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu ${OS_CODENAME} stable" \
  | ${SUDO[@]} tee /etc/apt/sources.list.d/docker.list >/dev/null

${SUDO[@]} apt update

echo "Installing docker packages..."
${SUDO[@]} apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

echo "Enabling and starting docker service..."
${SUDO[@]} systemctl enable --now docker

# Determine a non-root user to add to the docker group. Prefer the user who
# invoked sudo (SUDO_USER), otherwise use the current user when the script is
# run without sudo. If run as root with no SUDO_USER, we skip adding a user and
# print instructions.
TARGET_USER=""
if [[ -n "${SUDO_USER-}" ]] && [[ "${SUDO_USER}" != "root" ]]; then
  TARGET_USER=${SUDO_USER}
elif [[ $(id -u) -ne 0 ]]; then
  TARGET_USER=$(id -un)
fi

if [[ -n "${TARGET_USER}" ]]; then
  echo "Adding user ${TARGET_USER} to 'docker' group..."
  ${SUDO[@]} groupadd -f docker
  ${SUDO[@]} usermod -aG docker "${TARGET_USER}"
  # If not running as root, activate new group in current shell
  if [[ $(id -u) -ne 0 ]]; then
    echo "Activating docker group for current shell (newgrp docker)..."
    newgrp docker <<EONG
echo "Current user now in docker group."
EONG
  else
    echo "You may need to log out and back in, or run 'newgrp docker', to use docker without sudo."
  fi
else
  echo "No non-root invoking user detected; skipping automatic docker group add."
  echo "If you want to use docker without sudo, run as a privileged user and re-run this script, or run as root:"
  echo "  sudo usermod -aG docker <your-username>"
fi

echo "Docker installation completed. Quick smoke-test:"
echo "  sudo docker run --rm hello-world"
echo "If you added your user to the 'docker' group, log out and back in (or run 'newgrp docker') to use docker without sudo."

exit 0
