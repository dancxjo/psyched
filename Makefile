.PHONY: help ros2 build bootstrap update say install-services uninstall-services update-services start-services stop-services status-services

# Use bash for richer shell features where needed
SHELL := /bin/bash

help:
	@echo "Targets:"
	@echo "  ros2               - Install ROS 2 using tools/install_ros2.sh"
	@echo "  build              - Resolve deps with rosdep, colcon build, and re-source env"
	@echo "  bootstrap          - Run initial provisioning via tools/provision/bootstrap.sh"
	@echo "  update             - git pull then run bootstrap"
	@echo "  say                - Publish text to /voice topic (usage: make say TEXT=\"Hello world\")"
	@echo ""
	@echo "Service Management:"
	@echo "  install-services   - Install systemd services for enabled modules"
	@echo "  uninstall-services - Remove all psyched systemd services"
	@echo "  update-services    - Uninstall and reinstall services (for updates)"
	@echo "  start-services     - Start all enabled module services"
	@echo "  stop-services      - Stop all psyched services"
	@echo "  status-services    - Show status of all psyched services"
	@echo ""
	@echo "Examples:"
	@echo "  make ros2"
	@echo "  ./modules/foot/setup.sh && make build"
	@echo "  ./modules/voice/setup.sh && make build"
	@echo "  make say TEXT=\"Hello, this is a test\""
	@echo "  sudo make install-services"
	@echo "  sudo make start-services"

# Install ROS 2 via the provisioning script. You can override the distro:
#   make ros2 ROS_DISTRO=kilted
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
		eval "$$(SETUP_ENV_MODE=print ./tools/setup_env.sh)"; \
		echo "[build] Updating rosdep..."; \
		rosdep update; \
		echo "[build] Installing dependencies with rosdep..."; \
		cd src && rosdep install --from-paths . --ignore-src -r -y; \
		cd ..; \
		echo "[build] Running colcon build..."; \
		colcon build --symlink-install; \
	echo "[build] Re-sourcing environment..."; \
	eval "$$(SETUP_ENV_MODE=print ./tools/setup_env.sh)"; \
		echo "[build] Done."'

# Bootstrap the host/dev environment
# Usage:
#   make bootstrap
bootstrap:
	@bash -lc 'set -euo pipefail; \
		echo "[bootstrap] Running tools/provision/bootstrap.sh..."; \
		./tools/provision/bootstrap.sh; \
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
		if [ $$STASH_CREATED -eq 1 ]; then \
			echo "[update] Restoring stashed changes..."; \
			if ! git stash pop; then \
				echo "[update] Conflict when popping stash. Resolve conflicts then re-run make bootstrap."; \
				exit 1; \
			fi; \
		fi; \
		echo "[update] Running bootstrap..."; \
		$(MAKE) bootstrap; \
		echo "[update] Done."'

# Publish text to the /voice topic for text-to-speech
# Usage:
#   make say TEXT="Hello world"
#   make say TEXT="This is a longer message to speak"
say:
	@if [ -z "$(TEXT)" ]; then \
		echo "Error: Please provide TEXT parameter. Usage: make say TEXT=\"Your message here\""; \
		exit 1; \
	fi
	@bash -lc 'set -euo pipefail; \
		echo "[say] Sourcing ROS environment..."; \
		eval "$$(SETUP_ENV_MODE=print ./tools/setup_env.sh)"; \
		echo "[say] Publishing \"$(TEXT)\" to /voice topic..."; \
		ros2 topic pub --once /voice std_msgs/msg/String "data: \"$(TEXT)\""; \
		echo "[say] Message published."'

# Install systemd services for enabled modules
# Usage:
#   sudo make install-services
#   sudo make install-services HOST=cerebellum
install-services:
	@echo "[services] Installing systemd services for enabled modules..."
	@sudo -E ./tools/manage_services.sh install-enabled $(HOST)
	@echo "[services] Services installed and enabled."

# Remove all psyched systemd services
# Usage:
#   sudo make uninstall-services
uninstall-services:
	@echo "[services] Removing all psyched systemd services..."
	@sudo -E ./tools/manage_services.sh uninstall-all
	@echo "[services] All services removed."

# Update services (uninstall then reinstall) - useful after code updates
# Usage:
#   sudo make update-services
#   sudo make update-services HOST=cerebellum
update-services:
	@echo "[services] Updating systemd services..."
	@sudo -E ./tools/manage_services.sh uninstall-all
	@sudo -E ./tools/manage_services.sh install-enabled $(HOST)
	@echo "[services] Services updated."

# Start all enabled module services
# Usage:
#   sudo make start-services
#   sudo make start-services HOST=cerebellum
start-services:
	@echo "[services] Starting enabled module services..."
	@sudo -E ./tools/manage_services.sh start-enabled $(HOST)
	@echo "[services] Services started."

# Stop all psyched services
# Usage:
#   sudo make stop-services
stop-services:
	@echo "[services] Stopping all psyched services..."
	@sudo -E ./tools/manage_services.sh stop-all
	@echo "[services] Services stopped."

# Show status of all psyched services
# Usage:
#   make status-services
status-services:
	@echo "[services] Status of all psyched services:"
	@./tools/manage_services.sh list
