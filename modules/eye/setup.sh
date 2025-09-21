#!/usr/bin/env bash
set -euo pipefail

# Config: source ../../config/eye.env from real script location
REAL_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$REAL_PATH")"
MODULE_NAME="$(basename "$(dirname "$REAL_PATH")")"
CONF_FILE="$(cd "$SCRIPT_DIR/../.." && pwd)/config/${MODULE_NAME}.env"
if [ -f "$CONF_FILE" ]; then
    # shellcheck disable=SC1090
    . "$CONF_FILE"
fi

# Determine repository root regardless of current working directory
if REPO_DIR_GIT_ROOT=$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel 2>/dev/null); then
    REPO_DIR="$REPO_DIR_GIT_ROOT"
else
    # Fallback: modules/<name> is two levels below repo root
    REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
fi
SOURCE_DIR="${REPO_DIR}/src"

KINECT_ROS2_REPO="https://github.com/bribribriambriguy/kinect_ros2.git"
KINECT_ROS2_BRANCH="frame_correction"
LIBFREENECT_REPO="https://github.com/OpenKinect/libfreenect"

if ! command -v git >/dev/null 2>&1; then
    echo "Error: 'git' is required. Install Git from https://git-scm.com/downloads" >&2
    exit 1
fi

mkdir -p "${SOURCE_DIR}"
echo "[eye/setup] Using repo root: ${REPO_DIR}"
echo "[eye/setup] Populating source dir: ${SOURCE_DIR}"

# Clean up any stale module-local clones that could cause duplicate packages
LOCAL_MODULE_SRC_DIR="${SCRIPT_DIR}/src"
if [ -d "${LOCAL_MODULE_SRC_DIR}" ]; then
    for d in kinect_ros2 libfreenect; do
        if [ -d "${LOCAL_MODULE_SRC_DIR}/$d" ]; then
            echo "[eye/setup] Removing stale module-local clone: ${LOCAL_MODULE_SRC_DIR}/$d (to avoid colcon duplicates)"
            rm -rf "${LOCAL_MODULE_SRC_DIR}/$d"
        fi
    done
    rmdir "${LOCAL_MODULE_SRC_DIR}" 2>/dev/null || true
fi

# Clone kinect_ros2 (specific branch) if missing
if [ ! -d "${SOURCE_DIR}/kinect_ros2/.git" ]; then
    echo "[eye/setup] Cloning kinect_ros2 (branch: ${KINECT_ROS2_BRANCH})..."
    git clone --branch "${KINECT_ROS2_BRANCH}" --single-branch "${KINECT_ROS2_REPO}" "${SOURCE_DIR}/kinect_ros2"
else
    echo "[eye/setup] kinect_ros2 already present. Skipping clone."
fi

# Clone libfreenect if missing
if [ ! -d "${SOURCE_DIR}/libfreenect/.git" ]; then
    echo "[eye/setup] Cloning libfreenect..."
    git clone "${LIBFREENECT_REPO}" "${SOURCE_DIR}/libfreenect"
else
    echo "[eye/setup] libfreenect already present. Skipping clone."
fi

patch_cmake_include_usr() {
    local cmake_file="$1"
    if [ ! -f "$cmake_file" ]; then
        return 0
    fi
    if grep -Eq "^[[:space:]]*include_directories\(.*(/usr/include)" "$cmake_file"; then
        return 0
    fi
    echo "[eye/setup] Patching $cmake_file to include /usr/include"
    # Try to insert after first 'project(' line; if not found, after first 'cmake_minimum_required'
    if grep -n "^[[:space:]]*project\(" "$cmake_file" >/dev/null; then
        local line
        line=$(grep -n "^[[:space:]]*project\(" "$cmake_file" | head -n1 | cut -d: -f1)
        awk -v ln="$line" 'NR==ln{print; print "include_directories(/usr/include)"; next}1' "$cmake_file" >"$cmake_file.tmp" && mv "$cmake_file.tmp" "$cmake_file"
    elif grep -n "^[[:space:]]*cmake_minimum_required\(" "$cmake_file" >/dev/null; then
        local line
        line=$(grep -n "^[[:space:]]*cmake_minimum_required\(" "$cmake_file" | head -n1 | cut -d: -f1)
        awk -v ln="$line" 'NR==ln{print; print "include_directories(/usr/include)"; next}1' "$cmake_file" >"$cmake_file.tmp" && mv "$cmake_file.tmp" "$cmake_file"
    else
        # Fallback: append at end
        printf "\ninclude_directories(/usr/include)\n" >>"$cmake_file"
    fi
}

# Apply patch to CMakeLists of both repos if present
patch_cmake_include_usr "${SOURCE_DIR}/kinect_ros2/CMakeLists.txt"
patch_cmake_include_usr "${SOURCE_DIR}/libfreenect/CMakeLists.txt"

# Build and install libfreenect so that kinect_ros2 can link against it.
echo "[eye/setup] Building libfreenect..."
(
    set -euo pipefail
    cd "${SOURCE_DIR}/libfreenect"
    mkdir -p build
    cd build
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr ..
    make -j"${NPROC:-$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 1)}"
    if command -v sudo >/dev/null 2>&1; then
        sudo make install
    else
        make install
    fi
)

echo "[eye/setup] libfreenect build and install complete."

echo "Eye module setup complete. Run: make build"
