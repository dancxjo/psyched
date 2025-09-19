#!/usr/bin/env bash
set -euo pipefail

# Foot module setup: clone required repos into the workspace src/ and build via `psh build`.

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

# Build the workspace using psh (rosdep + colcon)
psh build

echo "Foot module setup complete."
