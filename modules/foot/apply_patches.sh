#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
SRC_DIR="${PSYCHED_WORKSPACE_SRC:-${REPO_ROOT}/work/src}"

echo "==> Applying patches for foot module"

PATCH_DIR="${SCRIPT_DIR}/patches"
mapfile -t PATCH_FILES < <(find "${PATCH_DIR}" -maxdepth 1 -type f -name '*.patch' | sort)

if [[ ${#PATCH_FILES[@]} -eq 0 ]]; then
  echo "--> No patches to apply"
  echo "✓ Patches applied"
  exit 0
fi

for patch_path in "${PATCH_FILES[@]}"; do
  echo "--> Applying $(basename "${patch_path}")"
  patch --forward -p0 -d "${SRC_DIR}" < "${patch_path}" || {
    status=$?
    if [[ ${status} -eq 1 ]]; then
      echo "    Patch already applied; skipping"
    else
      exit ${status}
    fi
  }
  rm -f "${SRC_DIR}/"*.rej
  rm -f "${SRC_DIR}/"*.orig
  echo "    Applied"
done

echo "✓ Patches applied"
