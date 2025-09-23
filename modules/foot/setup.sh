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


case "${ROS_DISTRO}" in
    jade|kinetic|lunar|melodic|noetic)
        echo "Skipping end-of-life ROS distro: ${ROS_DISTRO}"
        exit 0
        ;;
    jazzy|kilted|rolling)
        echo "Detected supported ROS2 distro: ${ROS_DISTRO}"
        ;;
    *)
        echo "Proceeding with generic ROS2 distro: ${ROS_DISTRO}"
        ;;
esac
# Determine repository root regardless of current working directory
if REPO_DIR_GIT_ROOT=$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel 2>/dev/null); then
        REPO_DIR="$REPO_DIR_GIT_ROOT"
else
        # Fallback: modules/<name> is two levels below repo root
        REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
fi
SOURCE_DIR="${REPO_DIR}/src"

CREATE_ROBOT_REPO="https://github.com/autonomylab/create_robot.git"
LIBCREATE_REPO="https://github.com/revyos-ros/libcreate.git"
LIBCREATE_BRANCH="fix-std-string"

if ! command -v git >/dev/null 2>&1; then
    echo "Error: 'git' is required. Install Git from https://git-scm.com/downloads" >&2
    exit 1
fi

mkdir -p "${SOURCE_DIR}"
echo "[foot/setup] Using repo root: ${REPO_DIR}"
echo "[foot/setup] Populating source dir: ${SOURCE_DIR}"

# Clean up any stale module-local clones that could cause duplicate packages
LOCAL_MODULE_SRC_DIR="${SCRIPT_DIR}/src"
if [ -d "${LOCAL_MODULE_SRC_DIR}" ]; then
    for d in create_robot libcreate; do
        if [ -d "${LOCAL_MODULE_SRC_DIR}/$d" ]; then
            echo "[foot/setup] Removing stale module-local clone: ${LOCAL_MODULE_SRC_DIR}/$d (to avoid colcon duplicates)"
            rm -rf "${LOCAL_MODULE_SRC_DIR}/$d"
        fi
    done
    rmdir "${LOCAL_MODULE_SRC_DIR}" 2>/dev/null || true
fi

# Clean up any stale module-local clones that could cause duplicate packages
LOCAL_MODULE_SRC_DIR="${SCRIPT_DIR}/src"
if [ -d "${LOCAL_MODULE_SRC_DIR}" ]; then
    for d in create_robot libcreate; do
        if [ -d "${LOCAL_MODULE_SRC_DIR}/$d" ]; then
            echo "[foot/setup] Removing stale module-local clone: ${LOCAL_MODULE_SRC_DIR}/$d (to avoid colcon duplicates)"
            rm -rf "${LOCAL_MODULE_SRC_DIR}/$d"
        fi
    done
    rmdir "${LOCAL_MODULE_SRC_DIR}" 2>/dev/null || true
fi

# Clone create_robot if missing
if [ ! -d "${SOURCE_DIR}/create_robot/.git" ]; then
    git clone "${CREATE_ROBOT_REPO}" "${SOURCE_DIR}/create_robot"
fi

# Clone libcreate (specific branch) if missing
if [ ! -d "${SOURCE_DIR}/libcreate/.git" ]; then
    git clone --branch "${LIBCREATE_BRANCH}" --single-branch "${LIBCREATE_REPO}" "${SOURCE_DIR}/libcreate"
fi

echo "Foot module setup complete."
