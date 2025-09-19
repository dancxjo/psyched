.PHONY: help ros2 build

# Use bash for richer shell features where needed
SHELL := /bin/bash

help:
	@echo "Targets:"
	@echo "  ros2   - Install ROS 2 using tools/install_ros2.sh"
	@echo "  build  - Resolve deps with rosdep, colcon build, and re-source env"
	@echo ""
	@echo "The legacy Makefile workflow has moved to psh."
	@echo "Examples:"
	@echo "  psh ros2"
	@echo "  psh module setup foot"
	@echo "  psh module remove foot"

# Install ROS 2 via the provisioning script. You can override the distro:
#   make ros2 ROS_DISTRO=jazzy
ros2:
	@bash -lc 'set -euo pipefail; ./tools/install_ros2.sh'

# Build the workspace:
#   1) Ensure ROS 2 environment is sourced (via tools/setup_env.sh)
#   2) rosdep update + install dependencies from ./src
#   3) colcon build (using symlink install for fast dev cycles)
#   4) Re-source environment so new packages are available in this shell
# Note: colcon does not have a separate "install" subcommand; the build step
#       populates the install/ tree. We emulate the requested flow accordingly.
build:
	@bash -lc 'set -euo pipefail; \
		echo "[build] Sourcing ROS environment..."; \
		eval "$$(PSH_ENV_MODE=print ./tools/setup_env.sh)"; \
		echo "[build] Updating rosdep..."; \
		rosdep update; \
		echo "[build] Installing dependencies with rosdep..."; \
		rosdep install --from-paths src --ignore-src -r -y; \
		echo "[build] Running colcon build..."; \
		colcon build --symlink-install; \
	echo "[build] Re-sourcing environment..."; \
	eval "$$(PSH_ENV_MODE=print ./tools/setup_env.sh)"; \
		echo "[build] Done."'
