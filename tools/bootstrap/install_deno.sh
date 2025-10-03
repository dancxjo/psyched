#!/usr/bin/env bash
#
# install_deno.sh - Install the Deno runtime on Debian/Ubuntu hosts.
# Uses GitHub release archives so it works in offline/air-gapped CI mirrors
# that proxy GitHub. Set DENO_VERSION to pin an alternate version.
#
# NOTE: Prefer running 'psh host setup' which now provides a Deno-based installer pipeline.

set -euo pipefail

VERSION=${DENO_VERSION:-v1.46.3}
ARCH=$(uname -m)

case "${ARCH}" in
  x86_64|amd64)
    TARGET="x86_64-unknown-linux-gnu"
    ;;
  aarch64|arm64)
    TARGET="aarch64-unknown-linux-gnu"
    ;;
  *)
    echo "Unsupported architecture: ${ARCH}" >&2
    exit 1
    ;;
 esac

SUDO=(sudo)
if [[ $(id -u) -eq 0 ]]; then
  SUDO=()
elif ! command -v sudo >/dev/null 2>&1; then
  echo "This script requires root privileges or sudo to escalate." >&2
  exit 1
fi

if command -v deno >/dev/null 2>&1; then
  echo "deno already installed: $(deno --version | head -n 1)"
  exit 0
fi

if ! command -v unzip >/dev/null 2>&1; then
  echo "Installing unzip dependency..."
  ${SUDO[@]} apt update
  ${SUDO[@]} apt install -y unzip
fi

TMPDIR=$(mktemp -d)
trap 'rm -rf "${TMPDIR}"' EXIT

ARCHIVE="deno-${TARGET}.zip"
URL="https://github.com/denoland/deno/releases/download/${VERSION}/${ARCHIVE}"

echo "Downloading deno ${VERSION} for ${TARGET}..."
curl -fsSL "${URL}" -o "${TMPDIR}/${ARCHIVE}"

unzip -o "${TMPDIR}/${ARCHIVE}" -d "${TMPDIR}" >/dev/null

${SUDO[@]} install -m 0755 "${TMPDIR}/deno" /usr/local/bin/deno

echo "deno ${VERSION} installed to /usr/local/bin/deno"
/usr/local/bin/deno --version
