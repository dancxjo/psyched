#!/usr/bin/env bash
set -euo pipefail


# Config: source ../../config/foot.env from real script location
REAL_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$REAL_PATH")"
MODULE_NAME="$(basename "$(dirname "$REAL_PATH")")"
CONF_FILE="$(cd "$SCRIPT_DIR/../.." && pwd)/config/${MODULE_NAME}.env"
if [ -f "$CONF_FILE" ]; then
    # shellcheck disable=SC1090
    . "$CONF_FILE"
fi

# Shared module helpers
MODULE_LIB="$(cd "$SCRIPT_DIR/../.." && pwd)/tools/lib/module.sh"
if [ -f "$MODULE_LIB" ]; then
    # shellcheck disable=SC1090
    . "$MODULE_LIB"
fi


REPO_DIR="$(pwd)"
SOURCE_DIR="${REPO_DIR}/src"

CREATE_ROBOT_REPO="https://github.com/autonomylab/create_robot.git"
LIBCREATE_REPO="https://github.com/revyos-ros/libcreate.git"
LIBCREATE_BRANCH="fix-std-string"

if ! command -v git >/dev/null 2>&1; then
    echo "Error: 'git' is required. Install Git from https://git-scm.com/downloads" >&2
    exit 1
fi

mkdir -p "${SOURCE_DIR}"

# Clone create_robot if missing
if [ ! -d "${SOURCE_DIR}/create_robot/.git" ]; then
    git clone "${CREATE_ROBOT_REPO}" "${SOURCE_DIR}/create_robot"
fi

# Clone libcreate (specific branch) if missing
if [ ! -d "${SOURCE_DIR}/libcreate/.git" ]; then
    git clone --branch "${LIBCREATE_BRANCH}" --single-branch "${LIBCREATE_REPO}" "${SOURCE_DIR}/libcreate"
fi

echo "Foot module setup complete."
