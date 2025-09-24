#!/usr/bin/env bash
set -euo pipefail

REAL_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$REAL_PATH")"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SRC_DIR="${REPO_DIR}/src"

mkdir -p "$SRC_DIR"

# Install system packages required by py_trees_ros / development
# Note: this script will attempt to use sudo to install apt packages when
# necessary. In restricted environments or CI you may want to run these
# steps manually or adapt to your package manager.
SYSTEM_PKGS=(python3-pip git)
missing_pkgs=()
for p in "${SYSTEM_PKGS[@]}"; do
  if ! dpkg -s "$p" >/dev/null 2>&1; then
    missing_pkgs+=("$p")
  fi
done
if [ ${#missing_pkgs[@]} -ne 0 ]; then
  echo "The following system packages are missing and will be installed: ${missing_pkgs[*]}"
  if command -v sudo >/dev/null 2>&1; then
    sudo apt-get update
    sudo apt-get install -y "${missing_pkgs[@]}"
  else
    echo "sudo not available; please install: ${missing_pkgs[*]}" >&2
  fi
fi

# Ensure pip is available as pip for the system python3
if ! command -v pip >/dev/null 2>&1; then
  if command -v python3 >/dev/null 2>&1; then
    python3 -m pip install --user pip || true
  fi
fi

# Install py_trees and py_trees_ros. Use --break-system-packages to allow
# installations into distributions that use system-managed Python packages.
set +e
pip install py_trees --break-system-packages
ret=$?
if [ $ret -ne 0 ]; then
  echo "pip install py_trees failed with exit $ret" >&2
fi

# py_trees_ros may be published or available via GitHub; try PyPI first,
# otherwise fall back to the GitHub URL used previously by the project.
pip install py_trees_ros --break-system-packages || \
  pip install "git+https://github.com/splintered-reality/py_trees_ros.git" --break-system-packages
set -e

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
