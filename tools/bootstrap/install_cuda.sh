#!/usr/bin/env bash
#
# install_cuda.sh - Install NVIDIA drivers and CUDA toolkit on Debian/Ubuntu systems.
# The script configures NVIDIA's CUDA apt repository, installs the recommended
# driver, and pulls in the CUDA toolkit packages commonly required by our
# forebrain host.
#
# Usage: ./install_cuda.sh
#
# Notes:
# * Requires root privileges (or sudo) and network access to NVIDIA's repositories.
# * Automatically detects the Ubuntu version and selects the matching CUDA repo.
# * Leaves the CUDA samples under /usr/local/cuda once complete.
#
# NOTE: Prefer running 'psh host setup' which now provides a Deno-based installer pipeline.

set -euo pipefail

SUDO=(sudo)
if [[ $(id -u) -eq 0 ]]; then
  SUDO=()
elif ! command -v sudo >/dev/null 2>&1; then
  echo "This script requires root privileges or sudo to escalate." >&2
  exit 1
fi

if command -v nvidia-smi >/dev/null 2>&1 && command -v nvcc >/dev/null 2>&1; then
  echo "Detected existing NVIDIA driver and CUDA toolkit. Nothing to do."
  exit 0
fi

export LANG=en_US.UTF-8

${SUDO[@]} apt-get update
${SUDO[@]} apt-get install -y ca-certificates curl gnupg lsb-release software-properties-common

. /etc/os-release
if [[ "${ID}" != "ubuntu" ]]; then
  echo "Unsupported distribution: ${ID}. This installer currently targets Ubuntu." >&2
  exit 1
fi

VERSION_SUFFIX=${VERSION_ID//./}
REPO_ID="ubuntu${VERSION_SUFFIX}"
KEYRING_PATH="/etc/apt/keyrings/nvidia-cuda-keyring.gpg"
REPO_URL="https://developer.download.nvidia.com/compute/cuda/repos/${REPO_ID}/x86_64/"

if [[ -z "${VERSION_SUFFIX}" ]]; then
  echo "Unable to determine Ubuntu version from VERSION_ID=${VERSION_ID}" >&2
  exit 1
fi

${SUDO[@]} mkdir -p /etc/apt/keyrings
curl -fsSL "${REPO_URL}/3bf863cc.pub" | ${SUDO[@]} gpg --dearmor -o "${KEYRING_PATH}"
${SUDO[@]} chmod a+r "${KEYRING_PATH}"

echo "deb [signed-by=${KEYRING_PATH}] ${REPO_URL} /" | \
  ${SUDO[@]} tee /etc/apt/sources.list.d/cuda-${REPO_ID}.list >/dev/null

${SUDO[@]} apt-get update

CUDA_PACKAGES=(
  cuda-toolkit-12-4
  nvidia-driver-535
)

echo "Installing CUDA packages: ${CUDA_PACKAGES[*]}"
${SUDO[@]} apt-get install -y "${CUDA_PACKAGES[@]}"

echo "CUDA installation complete. Recommended post-install steps:"
echo "  - Reboot the system to load the NVIDIA kernel modules."
echo "  - Verify with 'nvidia-smi' and 'nvcc --version'."

echo "You can add /usr/local/cuda/bin to your PATH for convenience:"
echo "  echo 'export PATH=/usr/local/cuda/bin:$PATH' >> ~/.bashrc"

echo "Done."
