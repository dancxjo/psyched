.PHONY: help ros2 build bootstrap update

# Use bash for richer shell features where needed
SHELL := /bin/bash

help:
	@echo "Targets:"
	@echo "  ros2               - Install ROS 2 using tools/install_ros2.sh"
	@echo "  build              - Resolve deps with rosdep, colcon build, and re-source env"
	@echo "  bootstrap          - Run initial provisioning via tools/provision/bootstrap.sh"
	@echo "  update             - git pull then run bootstrap"
	@echo ""
	@echo "Examples:"
	@echo "  make ros2"
	@echo "  ./modules/foot/setup.sh && make build"
	@echo "  ./modules/voice/setup.sh && make build"

# Install ROS 2 via the provisioning script. You can override the distro:
#   make ros2 ROS_DISTRO=kilted
ros2:
	@bash -lc 'set -euo pipefail; \
		if [ -d /opt/ros ] && [ $$(ls /opt/ros/ | wc -l) -gt 0 ]; then \
			INSTALLED_DISTROS=$$(ls /opt/ros/ | tr "\n" " "); \
			echo "[ros2] ROS 2 is already installed, skipping..."; \
			echo "[ros2] Found installed distros: $$INSTALLED_DISTROS"; \
		else \
			echo "[ros2] Installing ROS 2..."; \
			./tools/install_ros2.sh; \
		fi'

# Build the workspace:
#   1) Ensure ROS 2 environment is sourced (via tools/setup_env.sh)
#   2) rosdep update + install dependencies from ./src
#   3) colcon build (using symlink install for fast dev cycles)
#   4) Re-source environment so new packages are available in this shell
# Note: colcon does not have a separate "install" subcommand; the build step
#       populates the install/ tree. We emulate the requested flow accordingly.
build:
	@bash -lc 'set -euo pipefail; \
		source /opt/ros/kilted/setup.bash; \
		rosdep update; \
		cd src && rosdep install --from-paths . --ignore-src -r -y; \
		cd ..; \
		colcon build --symlink-install; \
		source install/setup.bash; \
		echo "[build] Done."'

# Bootstrap the host/dev environment
# Usage:
#   make bootstrap
bootstrap:
	@bash -lc 'set -euo pipefail; \
		echo "[bootstrap] Running tools/provision/bootstrap.sh..."; \
		./tools/provision/bootstrap.sh; \
		echo "[bootstrap] Downloading popular piper voices..."; \
		$(MAKE) get-piper-voices VOICES=popular || echo "[bootstrap] Warning: Could not download piper voices, continuing..."; \
		echo "[bootstrap] Done."'

# Update the repository and re-run bootstrap
# Usage:
#   make update
update:
	@bash -lc 'set -euo pipefail; \
		echo "[update] Stashing local changes (including untracked)..."; \
		STASH_CREATED=0; \
		if ! git diff --quiet || ! git diff --cached --quiet || [ -n "$(shell git ls-files --others --exclude-standard)" ]; then \
			git stash push -u -m "pre-update" >/dev/null; \
			STASH_CREATED=1; \
		fi; \
		echo "[update] Pulling latest changes (rebase)..."; \
		git pull --rebase; \
		echo "[update] Running bootstrap..."; \
		$(MAKE) bootstrap; \
		echo "[update] Done."'
