echo "[setup_wifi] Running user-space Wi-Fi setup and ROS2 status publisher (ROS2 package)..."
#!/usr/bin/env bash
set -euo pipefail

# Determine real script location
REAL_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$REAL_PATH")"
MODULE_NAME="$(basename "$(dirname "$REAL_PATH")")"

# Determine repository root
if REPO_DIR_GIT_ROOT=$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel 2>/dev/null); then
	REPO_DIR="$REPO_DIR_GIT_ROOT"
else
	REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
fi
SOURCE_DIR="${REPO_DIR}/src"

# Link module-local packages into workspace src/
MODULE_PACKAGES_DIR="${SCRIPT_DIR}/packages"
if [ -d "$MODULE_PACKAGES_DIR" ]; then
	mkdir -p "$SOURCE_DIR"
	for PKG in "$MODULE_PACKAGES_DIR"/*; do
		PKG_NAME="$(basename "$PKG")"
		TARGET_LINK="${SOURCE_DIR}/${PKG_NAME}"
		if [ -e "$TARGET_LINK" ]; then
			echo "[wifi/setup] Removing stale src link: $TARGET_LINK"
			rm -rf "$TARGET_LINK"
		fi
		echo "[wifi/setup] Linking $PKG -> $TARGET_LINK"
		ln -s "$PKG" "$TARGET_LINK"
	done
else
	echo "[wifi/setup] No packages/ directory found in $SCRIPT_DIR; skipping src linking"
fi

echo "[setup_wifi] Running user-space Wi-Fi setup and ROS2 status publisher (ROS2 package)..."
ros2 run wifi wifi_setup
