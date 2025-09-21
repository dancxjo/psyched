.PHONY: help ros2 build bootstrap update bringup setup shutdown \
	 systemd-generate systemd-install systemd-reload \
	 systemd-enable systemd-disable systemd-start systemd-stop systemd-status \
	 systemd-debug

# Use bash for richer shell features where needed
SHELL := /bin/bash

help:
	@echo "Targets:"
	@echo "  ros2               - Install ROS 2 using tools/install_ros2.sh"
	@echo "  build              - Resolve deps with rosdep, colcon build, and re-source env"
	@echo "  bootstrap          - Run initial provisioning via tools/provision/bootstrap.sh"
	@echo "  update             - git pull then run bootstrap"
	@echo "  bringup            - Launch all host modules in background via ./tools/bringup"
	@echo "  setup              - Run setup.sh for all host modules via ./tools/setup"
	@echo "  shutdown           - Stop modules (run shutdown.sh) via ./tools/stop"
	@echo "  systemd-generate   - Generate host unit files under hosts/$(shell hostname -s)/systemd"
	@echo "  systemd-install    - Copy generated units to /etc/systemd/system and daemon-reload"
	@echo "  systemd-enable     - Enable and start units (UNIT=name.service to limit)"
	@echo "  systemd-disable    - Disable and stop units (UNIT=name.service to limit)"
	@echo "  systemd-start      - Start units (UNIT=name.service to limit)"
	@echo "  systemd-stop       - Stop units (UNIT=name.service to limit)"
	@echo "  systemd-status     - Show status of units (UNIT=name.service to limit)"
	@echo "  systemd-reload     - Run systemctl daemon-reload"
	@echo "  systemd-debug      - Debug a unit: cat config, status, and tail logs (UNIT=...)"
	@echo ""
	@echo "Examples:"
	@echo "  make ros2"
	@echo "  ./modules/foot/setup.sh && make build"
	@echo "  ./modules/voice/setup.sh && make build"
	@echo "  make systemd-generate && sudo make systemd-install"
	@echo "  make systemd-enable UNIT=psyched-pilot.service"
	@echo "  make systemd-debug UNIT=psyched-pilot.service"

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
	@bash -lc 'set -e; \
		source ./tools/setup_env.sh; \
		rosdep update; \
		cd src && rosdep install --from-paths . --ignore-src -r -y; \
		cd ..; \
		colcon build --symlink-install; \
		ln -sfn $(CURDIR)/hosts $(CURDIR)/install/hosts; \
		. install/setup.bash || true; \
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
		echo "[update] Running bootstrap..."; \
		$(MAKE) bootstrap; \
		echo "[update] Done."'

bringup:
	@bash -lc 'set -eo pipefail; ./tools/bringup'

setup:
	@bash -lc 'set -eo pipefail; rm -rf ./src && mkdir -p src && ./tools/setup'

shutdown:
	@bash -lc 'set -eo pipefail; ./tools/stop'

# -----------------------------
# Systemd helpers
# -----------------------------

# Override these on the command line if needed, e.g.:
#   make systemd-install SYSTEMD_DIR=/etc/systemd/system HOST_SHORT=myhost
HOST_SHORT ?= $(shell hostname -s)
HOST_UNITS_DIR := hosts/$(HOST_SHORT)/systemd
SYSTEMD_DIR ?= /etc/systemd/system
# Space-separated list of service basenames present under host units dir
UNITS := $(shell [ -d $(HOST_UNITS_DIR) ] && find $(HOST_UNITS_DIR) -maxdepth 1 -type f -name 'psyched-*.service' -printf '%f ' 2>/dev/null || true)
# Use UNIT variable if provided, else operate on all host units
UNIT_LIST := $(if $(UNIT),$(UNIT),$(UNITS))

systemd-generate:
	@bash -lc 'set -euo pipefail; \
		SERVICE_USER=$${SERVICE_USER:-$$USER} SYSTEMD_DIR=$(SYSTEMD_DIR) ./tools/systemd_generate'

systemd-install:
	@bash -lc 'set -euo pipefail; \
		shopt -s nullglob; files=($(HOST_UNITS_DIR)/*.service); shopt -u nullglob; \
		if (( $${#files[@]} )); then \
			echo "[systemd-install] Installing $${#files[@]} unit(s) to $(SYSTEMD_DIR)..."; \
			sudo cp $(HOST_UNITS_DIR)/*.service $(SYSTEMD_DIR)/; \
			sudo systemctl daemon-reload; \
			echo "[systemd-install] Done."; \
		else \
			echo "[systemd-install] No unit files found under $(HOST_UNITS_DIR). Run make systemd-generate first."; \
		fi'

systemd-reload:
	@bash -lc 'set -euo pipefail; sudo systemctl daemon-reload'

systemd-enable:
	@bash -lc 'set -euo pipefail; \
		if [ -z "$(UNIT_LIST)" ]; then echo "[systemd-enable] No units to enable."; exit 0; fi; \
		echo "[systemd-enable] Enabling: $(UNIT_LIST)"; \
		sudo systemctl enable --now $(UNIT_LIST)'

systemd-disable:
	@bash -lc 'set -euo pipefail; \
		if [ -z "$(UNIT_LIST)" ]; then echo "[systemd-disable] No units to disable."; exit 0; fi; \
		echo "[systemd-disable] Disabling: $(UNIT_LIST)"; \
		sudo systemctl disable --now $(UNIT_LIST) || true'

systemd-start:
	@bash -lc 'set -euo pipefail; \
		if [ -z "$(UNIT_LIST)" ]; then echo "[systemd-start] No units to start."; exit 0; fi; \
		echo "[systemd-start] Starting: $(UNIT_LIST)"; \
		sudo systemctl start $(UNIT_LIST)'

systemd-stop:
	@bash -lc 'set -euo pipefail; \
		if [ -z "$(UNIT_LIST)" ]; then echo "[systemd-stop] No units to stop."; exit 0; fi; \
		echo "[systemd-stop] Stopping: $(UNIT_LIST)"; \
		sudo systemctl stop $(UNIT_LIST) || true'

systemd-status:
	@bash -lc 'set -euo pipefail; \
		if [ -z "$(UNIT_LIST)" ]; then echo "[systemd-status] No units found under $(HOST_UNITS_DIR)."; exit 0; fi; \
		echo "[systemd-status] Status for: $(UNIT_LIST)"; \
		systemctl --no-pager --full status $(UNIT_LIST) || true'

# Debug a unit: show config, status, and tail logs
# Usage: make systemd-debug UNIT=psyched-voice.service
systemd-debug:
	@bash -lc 'set -euo pipefail; \
		if [ -z "$(UNIT)" ]; then \
			echo "Usage: make systemd-debug UNIT=psyched-<module>.service"; \
			echo "Available units: $(UNITS)"; \
			exit 1; \
		fi; \
		echo "[systemd-debug] ==== systemctl cat $(UNIT) ===="; \
		systemctl cat $(UNIT) || true; \
		echo "[systemd-debug] ==== systemctl status $(UNIT) ===="; \
		systemctl --no-pager --full status $(UNIT) || true; \
		echo "[systemd-debug] ==== journalctl -u $(UNIT) (last 200 lines, follow) ===="; \
		sudo journalctl -u $(UNIT) -n 200 -f'

