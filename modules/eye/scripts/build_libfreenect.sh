#!/usr/bin/env bash
set -euo pipefail

# Build and install libfreenect from the synced workspace checkout.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
SRC_DIR="${PSYCHED_WORKSPACE_SRC:-${REPO_ROOT}/work/src}"
LIBFREENECT_DIR="${SRC_DIR}/libfreenect"

if [[ ! -d "${LIBFREENECT_DIR}" ]]; then
  echo "[eye/libfreenect] Source directory not found at ${LIBFREENECT_DIR}; skipping build"
  exit 0
fi

mkdir -p "${LIBFREENECT_DIR}/build"
cd "${LIBFREENECT_DIR}/build"

cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr -DBUILD_EXAMPLES=OFF -DBUILD_FAKENECT=OFF ..

if command -v nproc >/dev/null 2>&1; then
  JOBS=$(nproc)
else
  JOBS=$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 1)
fi
make -j"${JOBS:-1}"

SUDO_CMD=""
if command -v sudo >/dev/null 2>&1 && [[ ${EUID:-0} -ne 0 ]]; then
  SUDO_CMD="sudo"
fi

if [[ -n ${SUDO_CMD} ]]; then
  INSTALL_CMD=("${SUDO_CMD}" make install)
else
  INSTALL_CMD=(make install)
fi

if ! "${INSTALL_CMD[@]}"; then
  echo "[eye/libfreenect] make install failed; continuing without system install" >&2
fi
