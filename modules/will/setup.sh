#!/usr/bin/env bash
set -euo pipefail

REAL_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$REAL_PATH")"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SRC_DIR="${REPO_DIR}/src"

mkdir -p "$SRC_DIR"

echo "Linking will module packages into workspace src"
for pkg in "$SCRIPT_DIR"/packages/*; do
  if [ -d "$pkg" ]; then
    pkg_name="$(basename "$pkg")"
    target="${SRC_DIR}/${pkg_name}"
    if [ -e "$target" ]; then
      echo " - $pkg_name already linked"
    else
      ln -s "$pkg" "$target"
      echo " - linked $pkg_name"
    fi
  fi
done

echo "will module setup complete"
