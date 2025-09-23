#!/usr/bin/env bash
set -euo pipefail

echo "Setting up nav module..."

# Compute repository root (one level above "modules")
REPO_DIR="$(dirname "$(dirname "$(dirname "$(realpath "$0")")")")"
SCRIPT_DIR="$(dirname "$(realpath "$0")")"
MODULE_DIR="$SCRIPT_DIR"
SRC_DIR="$REPO_DIR/src"
echo "Repo dir: $REPO_DIR"
echo "Module dir: $MODULE_DIR"

# Ensure workspace src exists up front so optional Nav2 linking works.
mkdir -p "$SRC_DIR"

if [ "${EUID}" -ne 0 ]; then
  SUDO=sudo
else
  SUDO=""
fi

if ! command -v apt-get >/dev/null 2>&1; then
  echo "apt-get not found — this setup script only supports Debian/Ubuntu hosts."
  exit 1
fi

if ! ping -c1 -W1 8.8.8.8 >/dev/null 2>&1; then
  echo "Network seems unavailable. Skipping package install and repo clones."
  echo "Please install nav2 and related packages manually when online."
  exit 0
fi

${SUDO} apt-get update

# Determine ROS distro; many repos expect one to be set. Default to `humble` if unset.
ROS_DISTRO="${ROS_DISTRO:-humble}"
echo "Using ROS distro: ${ROS_DISTRO}"

# Candidate APT packages for Nav2 core stacks with AMCL. Mapping stacks like
# RTAB-Map are intentionally excluded from this module.
declare -a CANDIDATES=(
  "ros-${ROS_DISTRO}-nav2-bringup"
  "ros-${ROS_DISTRO}-nav2-amcl"
  "ros-${ROS_DISTRO}-nav2-controller"
  "ros-${ROS_DISTRO}-nav2-core"
  "ros-${ROS_DISTRO}-nav2-behavior-tree"
  "ros-${ROS_DISTRO}-nav2-costmap-2d"
  "ros-${ROS_DISTRO}-nav2-map-server"
  "ros-${ROS_DISTRO}-nav2-navfn-planner"
  "ros-${ROS_DISTRO}-nav2-smac-planner"
  "ros-${ROS_DISTRO}-nav2-lifecycle-manager"
  "ros-${ROS_DISTRO}-pcl-ros"
  "libpcl-dev"
)

PKGS_AVAILABLE=()
PKGS_MISSING=()

echo "Checking availability of candidate APT packages..."
for pkg in "${CANDIDATES[@]}"; do
  if apt-cache show "$pkg" >/dev/null 2>&1; then
    PKGS_AVAILABLE+=("$pkg")
  else
    PKGS_MISSING+=("$pkg")
  fi
done

if [ ${#PKGS_AVAILABLE[@]} -gt 0 ]; then
  echo "Installing available packages: ${PKGS_AVAILABLE[*]}"
  ${SUDO} apt-get install -y "${PKGS_AVAILABLE[@]}"
else
  echo "No nav-related APT packages were available for ${ROS_DISTRO}."
fi

if [ ${#PKGS_MISSING[@]} -gt 0 ]; then
  echo "The following candidate packages were not found in APT and were skipped:"
  for m in "${PKGS_MISSING[@]}"; do
    echo "  - $m"
  done
fi

# Optionally use a local Nav2 source tree instead of system package
# Set NAV2_LOCAL_DIR to a directory containing Nav2 source packages (or a meta-package)
# and set BUILD_NAV2=true to request building it before building 'nav'.
if [ -n "${NAV2_LOCAL_DIR:-}" ]; then
  if [ -d "$NAV2_LOCAL_DIR" ]; then
    echo "Using local Nav2 source from: $NAV2_LOCAL_DIR"
    # Link into workspace src so colcon will find it
    if [ -e "$SRC_DIR/nav2_local" ]; then
      echo "Local nav2 link $SRC_DIR/nav2_local already exists — skipping"
    else
      ln -s "$NAV2_LOCAL_DIR" "$SRC_DIR/nav2_local"
      echo "Linked local Nav2 sources into $SRC_DIR/nav2_local"
    fi
    # Optionally build local Nav2 first
    if [ "${BUILD_NAV2:-false}" = "true" ]; then
      if command -v colcon >/dev/null 2>&1; then
        echo "Building local Nav2 packages (this may take some time)..."
        colcon build --packages-select $(/bin/ls "$NAV2_LOCAL_DIR" | sed -e 's/\.//g' | tr '\n' ' ') --symlink-install || true
      else
        echo "colcon not found — cannot build local Nav2. Set BUILD_NAV2=false or install colcon."
      fi
    fi
  else
    echo "NAV2_LOCAL_DIR is set to '$NAV2_LOCAL_DIR' but that path does not exist or is not a directory."
  fi
fi

# Ensure workspace src exists and symlink module-local packages into it so colcon sees them

echo "Linking local module packages into $SRC_DIR"
for pkgpath in "$MODULE_DIR"/packages/*; do
  if [ -d "$pkgpath" ]; then
    # Detect ROS package: either package.xml (ament) or setup.py (python package)
    if [ -f "$pkgpath/package.xml" ] || [ -f "$pkgpath/setup.py" ]; then
      pkgname="$(basename "$pkgpath")"
      target="$SRC_DIR/$pkgname"
      if [ -e "$target" ]; then
        echo "Target $target already exists — skipping"
      else
        echo "Creating symlink: $target -> $pkgpath"
        ln -s "$pkgpath" "$target"
      fi
    fi
  fi
done

# Optionally build the local nav package (depth_projection and vision prompt) if requested.
# Set BUILD=true to trigger a targeted build. Default is true for convenience.
BUILD="${BUILD:-true}"
if [ "$BUILD" = "true" ]; then
  # Source ROS install if available
  if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    # Some ROS setup scripts reference variables that may be unset and would
    # trigger an error under 'set -u' in this script. Temporarily disable -u
    # while sourcing external setup files and restore it afterwards.
    set +u
    # shellcheck disable=SC1091
    . "/opt/ros/${ROS_DISTRO}/setup.bash"
    set -u
  else
    echo "/opt/ros/${ROS_DISTRO}/setup.bash not found — please ensure ROS2 is installed and sourced before building."
  fi

  # Source workspace install if it exists (incremental build)
  if [ -f "$REPO_DIR/install/setup.bash" ]; then
    # Temporarily disable -u for same reason as above when sourcing workspace setup
    set +u
    # shellcheck disable=SC1091
    . "$REPO_DIR/install/setup.bash"
    set -u
  fi

  echo "Running targeted colcon build for nav packages (this may take some time)..."
  # Prefer building only the psyched_nav package so the rest of the workspace isn't rebuilt unintentionally
  if command -v colcon >/dev/null 2>&1; then
    colcon build --packages-select psyched_nav --symlink-install
  else
    echo "colcon not found — skipping build. Install colcon and re-run to build the nav package."
  fi
fi

echo "Nav module setup complete."

