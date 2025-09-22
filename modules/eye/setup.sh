
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
    REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
fi
SOURCE_DIR="${REPO_DIR}/src"
ROS_DISTRO="${ROS_DISTRO:-kilted}"

ensure_cv_bridge_overlay() {
    local header="/opt/ros/${ROS_DISTRO}/include/cv_bridge/cv_bridge.hpp"
    local pkg="ros-${ROS_DISTRO}-cv-bridge"
    local repo="${SOURCE_DIR}/vision_opencv"
    if dpkg -s "$pkg" >/dev/null 2>&1 || [ -f "$header" ]; then
        echo "[eye/setup] cv_bridge available via package or header: $pkg or $header"
        return 0
    fi
    if [ ! -d "$repo" ]; then
        echo "[eye/setup] Cloning vision_opencv overlay to $repo"
        git clone https://github.com/ros-perception/vision_opencv.git "$repo"
    fi
    if [ -d "$repo/.git" ]; then
        git -C "$repo" fetch --tags --force >/dev/null 2>&1 || true
        if git -C "$repo" rev-parse --verify "$ROS_DISTRO" >/dev/null 2>&1; then
            git -C "$repo" checkout "$ROS_DISTRO" >/dev/null 2>&1 || true
        elif git -C "$repo" rev-parse --verify "ros2" >/dev/null 2>&1; then
            git -C "$repo" checkout "ros2" >/dev/null 2>&1 || true
        fi
    fi
    if [ -d "$repo" ]; then
        rm -f "$repo/COLCON_IGNORE" 2>/dev/null || true
    fi
}

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

# Ensure cv_bridge (vision_opencv) overlay exists when system package is missing
ensure_cv_bridge_overlay

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

# After cloning or when present, ensure kinect_ros2 CMakeLists references cv_bridge
ensure_kinect_cv_bridge() {
    local cmake_file="${SOURCE_DIR}/kinect_ros2/CMakeLists.txt"
    if [ ! -f "$cmake_file" ]; then
        echo "[eye/setup] kinect_ros2 CMakeLists not found at $cmake_file"
        return 0
    fi

    # Ensure find_package(cv_bridge REQUIRED) exists once
    if ! grep -E "^[[:space:]]*find_package\(cv_bridge" "$cmake_file" >/dev/null 2>&1; then
        echo "[eye/setup] Adding find_package(cv_bridge REQUIRED) to $cmake_file"
        # Append after the OpenCV or sensor_msgs find_package if present, otherwise after ament_cmake, else append
        if grep -n "find_package(OpenCV" "$cmake_file" >/dev/null 2>&1; then
            ln=$(grep -n "find_package(OpenCV" "$cmake_file" | head -n1 | cut -d: -f1)
            awk -v ln="$ln" 'NR==ln{print; print "find_package(cv_bridge REQUIRED)"; next}1' "$cmake_file" >"$cmake_file.tmp" && mv "$cmake_file.tmp" "$cmake_file"
        elif grep -n "find_package(sensor_msgs" "$cmake_file" >/dev/null 2>&1; then
            ln=$(grep -n "find_package(sensor_msgs" "$cmake_file" | head -n1 | cut -d: -f1)
            awk -v ln="$ln" 'NR==ln{print; print "find_package(cv_bridge REQUIRED)"; next}1' "$cmake_file" >"$cmake_file.tmp" && mv "$cmake_file.tmp" "$cmake_file"
        elif grep -n "find_package(ament_cmake" "$cmake_file" >/dev/null 2>&1; then
            ln=$(grep -n "find_package(ament_cmake" "$cmake_file" | head -n1 | cut -d: -f1)
            awk -v ln="$ln" 'NR==ln{print; print "find_package(cv_bridge REQUIRED)"; next}1' "$cmake_file" >"$cmake_file.tmp" && mv "$cmake_file.tmp" "$cmake_file"
        else
            printf "\nfind_package(cv_bridge REQUIRED)\n" >>"$cmake_file"
        fi
    else
        echo "[eye/setup] cv_bridge find_package already present in kinect_ros2 CMakeLists"
    fi

    # Helper: add an entry only if not present inside the ament_target_dependencies block
    add_dep_to_target() {
        local target="$1"
        local dep="$2"
        # If dependency already present for target, skip
        if grep -E "ament_target_dependencies\($target[[:space:]]*.*$dep" "$cmake_file" >/dev/null 2>&1; then
            return 0
        fi
        # Insert dep before the closing ')' of the target dependency block
        awk -v tgt="ament_target_dependencies($target" -v dep="  $dep" '
        BEGIN{inblock=0}
        index($0, tgt) {print; inblock=1; next}
        inblock && $0 ~ /\)/ {print dep; print; inblock=0; next}
        {print}
        ' "$cmake_file" >"$cmake_file.tmp" && mv "$cmake_file.tmp" "$cmake_file"
    }

    add_dep_to_target kinect_ros2_component cv_bridge
    add_dep_to_target kinect_ros2_node cv_bridge
}

# Ensure the kinect overlay has cv_bridge listed in its CMake files
ensure_kinect_cv_bridge

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
    local ros_cv_bridge_dir="/opt/ros/${ROS_DISTRO}/include/cv_bridge/cv_bridge"
    if grep -Fq "include_directories(/usr/include)" "$cmake_file" && grep -Fq "include_directories(${ros_cv_bridge_dir})" "$cmake_file"; then
        return 0
    fi
    echo "[eye/setup] Patching $cmake_file to include /usr/include and ${ros_cv_bridge_dir} after first project()"
    # Use awk to safely locate the first 'project(' line regardless of grep regex support
    local line
    line=$(awk '/project[[:space:]]*\(/ {print NR; exit}' "$cmake_file" || true)
    if [ -n "$line" ]; then
        awk -v ln="$line" -v rosdir="$ros_cv_bridge_dir" 'NR==ln{print; print "include_directories(/usr/include)"; print "include_directories("rosdir")"; next}1' "$cmake_file" > "$cmake_file.tmp" && mv "$cmake_file.tmp" "$cmake_file"
    else
        awk -v rosdir="$ros_cv_bridge_dir" 'NR==1{print "include_directories(/usr/include)"; print "include_directories("rosdir")"; print} NR>1{print}' "$cmake_file" > "$cmake_file.tmp" && mv "$cmake_file.tmp" "$cmake_file"
    fi
}

# Apply patch to CMakeLists of both repos if present
patch_cmake_include_usr "${SOURCE_DIR}/kinect_ros2/CMakeLists.txt"
patch_cmake_include_usr "${SOURCE_DIR}/libfreenect/CMakeLists.txt"


# Ensure all vision dependencies for Kinect and RGB-D pipelines
ensure_vision_deps() {
    if ! command -v apt-get >/dev/null 2>&1; then
        echo "[eye/setup] apt-get not available; skipping vision deps install"
        return 0
    fi
    sudo apt-get update -y >/dev/null 2>&1 || true
    sudo apt-get install -y \
        ros-${ROS_DISTRO}-cv-bridge \
        ros-${ROS_DISTRO}-camera-calibration-parsers \
        ros-${ROS_DISTRO}-image-transport \
        ros-${ROS_DISTRO}-image-transport-plugins \
        ros-${ROS_DISTRO}-image-pipeline \
        ros-${ROS_DISTRO}-perception \
        ros-${ROS_DISTRO}-perception-pcl \
        ros-${ROS_DISTRO}-vision-msgs \
        ros-${ROS_DISTRO}-usb-cam \
        libopencv-dev \
        python3-opencv \
        libglu1-mesa-dev \
        freeglut3-dev \
        mesa-common-dev \
        libogre-1.12-dev \
        ros-${ROS_DISTRO}-rviz2 \
        libgl1 \
        libegl1 \
        libxrandr2 \
        libxrandr-dev \
        libxinerama1 \
        libxinerama-dev \
        libxcursor1 \
        libxcursor-dev
}

# Call ensure_vision_deps before other system deps
ensure_vision_deps

# Ensure required system dependencies for libfreenect (libusb headers, pkg-config)
echo "[eye/setup] Ensuring system dependencies for libfreenect (libusb-1.0-0-dev, pkg-config)..."
if command -v apt-get >/dev/null 2>&1; then
        sudo apt-get update -y >/dev/null 2>&1 || true
        sudo apt-get install -y libusb-1.0-0-dev pkg-config >/dev/null
fi

install_ros_vision_packages() {
    local pkgs=("ros-${ROS_DISTRO}-cv-bridge" "ros-${ROS_DISTRO}-image-transport" "ros-${ROS_DISTRO}-vision-msgs")
    if ! command -v apt-get >/dev/null 2>&1; then
        echo "[eye/setup] apt-get not available; skipping ROS vision packages install: ${pkgs[*]}"
        return 0
    fi
    echo "[eye/setup] Installing ROS vision packages for distro: $ROS_DISTRO"
    sudo apt-get update -y >/dev/null 2>&1 || true
    local to_install=()
    for p in "${pkgs[@]}"; do
        if dpkg -s "$p" >/dev/null 2>&1; then
            echo "[eye/setup] Package already installed: $p"
        else
            to_install+=("$p")
        fi
    done
    if [ ${#to_install[@]} -eq 0 ]; then
        echo "[eye/setup] All ROS vision packages already installed."
        return 0
    fi
    echo "[eye/setup] Installing: ${to_install[*]}"
    sudo apt-get install -y "${to_install[@]}"
}

# Try to install ROS vision packages (cv_bridge, image_transport, vision_msgs)
install_ros_vision_packages

# Build and install libfreenect so that kinect_ros2 can link against it.
echo "[eye/setup] Building libfreenect..."
(
    set -euo pipefail
    cd "${SOURCE_DIR}/libfreenect"
    mkdir -p build
    cd build
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr -DBUILD_EXAMPLES=OFF -DBUILD_FAKENECT=OFF ..
    make -j"${NPROC:-$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 1)}"
    if command -v sudo >/dev/null 2>&1; then
        sudo make install
    else
        make install
    fi
)

echo "[eye/setup] libfreenect build and install complete."

echo "Eye module setup complete. Run: make build"
