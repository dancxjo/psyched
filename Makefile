.PHONY: help ros2 build bootstrap update say get-piper-voices install-services uninstall-services update-services start-services stop-services status-services diagnose-service logs-service

# Use bash for richer shell features where needed
SHELL := /bin/bash

help:
	@echo "Targets:"
	@echo "  ros2               - Install ROS 2 using tools/install_ros2.sh"
	@echo "  build              - Resolve deps with rosdep, colcon build, and re-source env"
	@echo "  bootstrap          - Run initial provisioning via tools/provision/bootstrap.sh"
	@echo "  update             - git pull then run bootstrap"
	@echo "  say                - Publish text to /voice topic (usage: make say TEXT=\"Hello world\")"
	@echo "  get-piper-voices   - Download additional Piper TTS voices (usage: make get-piper-voices VOICES=\"voice1,voice2\")"
	@echo ""
	@echo "Service Management:"
	@echo "  install-services   - Install systemd services for enabled modules"
	@echo "  uninstall-services - Remove all psyched systemd services"
	@echo "  update-services    - Uninstall and reinstall services (for updates)"
	@echo "  start-services     - Start all enabled module services (with detailed logging)"
	@echo "  stop-services      - Stop all psyched services"
	@echo "  status-services    - Show status of all psyched services"
	@echo "  diagnose-service   - Diagnose issues with a specific service (usage: make diagnose-service SERVICE=voice)"
	@echo "  logs-service       - Show recent logs for a service (usage: make logs-service SERVICE=voice [LINES=50])"
	@echo ""
	@echo "Examples:"
	@echo "  make ros2"
	@echo "  ./modules/foot/setup.sh && make build"
	@echo "  ./modules/voice/setup.sh && make build"
	@echo "  make say TEXT=\"Hello, this is a test\""
	@echo "  make get-piper-voices VOICES=\"en_US-amy-medium,en_US-ryan-high\""
	@echo "  sudo make install-services"
	@echo "  sudo make start-services"
	@echo "  make diagnose-service SERVICE=voice"
	@echo "  make logs-service SERVICE=voice LINES=100"

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
		echo "[bootstrap] Downloading popular piper voices..."; \
		$(MAKE) get-piper-voices VOICES=popular || echo "[bootstrap] Warning: Could not download piper voices, continuing..."; \
		echo "[bootstrap] Done."'

# Update the repository and re-run bootstrap
# Usage:
#   make update
update:
	@bash -lc 'set -euo pipefail; \
		echo "[update] Stopping old services..."; \
		$(MAKE) stop-services; \
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
		$(MAKE) install-services; \
		echo "[update] Starting services..."; \
		$(MAKE) start-services || { \
			echo "[update] Service start failed. Check diagnostics:"; \
			echo "  make diagnose-service SERVICE=<module_name>"; \
			echo "  make logs-service SERVICE=<module_name>"; \
			exit 1; \
		}; \
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

# Download additional Piper TTS voices from Hugging Face
# Usage:
#   make get-piper-voices VOICES="en_US-amy-medium,en_US-ryan-high"
#   make get-piper-voices VOICES="all"  # Downloads popular voices
#   PIPER_VOICES_DIR=/custom/path make get-piper-voices VOICES="en_US-amy-medium"
get-piper-voices:
	@bash -lc 'set -euo pipefail; \
		VOICES_DIR="$${PIPER_VOICES_DIR:-/opt/piper/voices}"; \
		echo "[get-piper-voices] Target directory: $$VOICES_DIR"; \
		mkdir -p "$$VOICES_DIR" || { echo "Error: Cannot create $$VOICES_DIR (try with sudo?)"; exit 1; }; \
		if [ -z "$(VOICES)" ]; then \
			echo "Usage: make get-piper-voices VOICES=\"voice1,voice2,...\""; \
			echo ""; \
			echo "Popular voices to try:"; \
			echo "  en_US-amy-medium     - Clear female voice"; \
			echo "  en_US-ryan-high      - Clear male voice"; \
			echo "  en_US-joe-medium     - Deep male voice"; \
			echo "  en_US-kathleen-low   - Warm female voice"; \
			echo "  en_US-kimberly-low   - Soft female voice"; \
			echo "  en_US-ljspeech-high  - High quality female"; \
			echo "  en_US-norman-medium  - Older male voice"; \
			echo "  en_US-arctic-medium  - Arctic voice dataset"; \
			echo ""; \
			echo "Or use: make get-piper-voices VOICES=\"popular\" for a curated selection"; \
			exit 1; \
		fi; \
		if [ "$(VOICES)" = "popular" ]; then \
			VOICE_LIST="en_US-amy-medium,en_US-ryan-high,en_US-joe-medium,en_US-kathleen-low"; \
		elif [ "$(VOICES)" = "all" ]; then \
			VOICE_LIST="en_US-amy-medium,en_US-ryan-high,en_US-joe-medium,en_US-kathleen-low,en_US-kimberly-low,en_US-ljspeech-high,en_US-norman-medium,en_US-arctic-medium"; \
		else \
			VOICE_LIST="$(VOICES)"; \
		fi; \
		echo "[get-piper-voices] Downloading voices: $$VOICE_LIST"; \
		IFS="," read -ra VOICE_ARRAY <<< "$$VOICE_LIST"; \
		for voice in "$${VOICE_ARRAY[@]}"; do \
			voice=$$(echo "$$voice" | xargs); \
			if [ -z "$$voice" ]; then continue; fi; \
			echo "[get-piper-voices] Processing voice: $$voice"; \
			MODEL_FILE="$$VOICES_DIR/$$voice.onnx"; \
			CONFIG_FILE="$$VOICES_DIR/$$voice.onnx.json"; \
			if [ -f "$$MODEL_FILE" ] && [ -f "$$CONFIG_FILE" ]; then \
				echo "[get-piper-voices] Voice $$voice already exists, skipping..."; \
				continue; \
			fi; \
			VOICE_PARTS=$$(echo "$$voice" | tr "-" " "); \
			LANG_COUNTRY=$$(echo "$$VOICE_PARTS" | cut -d" " -f1); \
			VOICE_NAME=$$(echo "$$VOICE_PARTS" | cut -d" " -f2); \
			QUALITY=$$(echo "$$VOICE_PARTS" | cut -d" " -f3); \
			LANG=$$(echo "$$LANG_COUNTRY" | cut -d"_" -f1); \
			COUNTRY=$$(echo "$$LANG_COUNTRY" | cut -d"_" -f2); \
			BASE_URL="https://huggingface.co/rhasspy/piper-voices/resolve/main/$$LANG/$$LANG_COUNTRY/$$VOICE_NAME/$$QUALITY"; \
			MODEL_URL="$$BASE_URL/$$voice.onnx?download=true"; \
			CONFIG_URL="$$BASE_URL/$$voice.onnx.json?download=true"; \
			echo "[get-piper-voices] Downloading $$voice.onnx..."; \
			if ! curl -fsSL -o "$$MODEL_FILE" "$$MODEL_URL"; then \
				echo "[get-piper-voices] Failed to download $$voice.onnx"; \
				rm -f "$$MODEL_FILE"; \
				continue; \
			fi; \
			echo "[get-piper-voices] Downloading $$voice.onnx.json..."; \
			if ! curl -fsSL -o "$$CONFIG_FILE" "$$CONFIG_URL"; then \
				echo "[get-piper-voices] Failed to download $$voice.onnx.json"; \
				rm -f "$$CONFIG_FILE"; \
				continue; \
			fi; \
			echo "[get-piper-voices] Successfully downloaded voice: $$voice"; \
		done; \
		echo "[get-piper-voices] Voice download complete!"; \
		echo "[get-piper-voices] Available voices in $$VOICES_DIR:"; \
		ls -1 "$$VOICES_DIR"/*.onnx 2>/dev/null | sed "s|$$VOICES_DIR/||g" | sed "s|\.onnx$$||g" || echo "  (none found)"; \
		echo "[get-piper-voices] To use a voice, set PIPER_MODEL environment variable or update your voice configuration."'

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

# Start all enabled module services (with detailed logging)
# Usage:
#   sudo make start-services
#   sudo make start-services HOST=cerebellum
start-services:
	@echo "[services] Starting enabled module services with detailed logging..."
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

# Diagnose issues with a specific service
# Usage:
#   make diagnose-service SERVICE=voice
#   make diagnose-service SERVICE=foot
diagnose-service:
	@if [ -z "$(SERVICE)" ]; then \
		echo "Error: Please provide SERVICE parameter. Usage: make diagnose-service SERVICE=voice"; \
		exit 1; \
	fi
	@echo "[services] Running diagnostics for service: $(SERVICE)"
	@./tools/manage_services.sh diagnose $(SERVICE)

# Show recent logs for a specific service
# Usage:
#   make logs-service SERVICE=voice
#   make logs-service SERVICE=foot LINES=50
logs-service:
	@if [ -z "$(SERVICE)" ]; then \
		echo "Error: Please provide SERVICE parameter. Usage: make logs-service SERVICE=voice"; \
		exit 1; \
	fi
	@echo "[services] Showing logs for service: $(SERVICE)"
	@./tools/manage_services.sh logs $(SERVICE) $(LINES)
