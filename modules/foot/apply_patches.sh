#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
SRC_DIR="${PSYCHED_WORKSPACE_SRC:-${REPO_ROOT}/work/src}"

echo "==> Applying patches for foot module"

PATCH_DIR="${SCRIPT_DIR}/patches"
PATCH_FILES=(
  "${PATCH_DIR}/create_msgs_add_rust_generator.patch"
)

for patch_path in "${PATCH_FILES[@]}"; do
  echo "--> Applying $(basename "${patch_path}")"
  patch --forward -p0 -d "${SRC_DIR}" < "${patch_path}" || {
    status=$?
    if [[ ${status} -eq 1 ]]; then
      echo "    Patch already applied; skipping"
    else
      exit ${status}
    fi
  };
  rm -f "${SRC_DIR}/"*.rej
  rm -f "${SRC_DIR}/"*.orig
  echo "    Applied"
done

echo "✓ Patches applied"
