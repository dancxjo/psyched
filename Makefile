#!/usr/bin/make -f

# Default ROS2 distribution
ROS_DISTRO ?= jazzy
WORKSPACE_PATH ?= /opt/psyched

.PHONY: help ros2 workspace env build clean

# Default target
help:
	@echo "Available targets:"
	@echo "  ros2      - Install ROS2 $(ROS_DISTRO) on the system"
	@echo "  workspace - Create and setup workspace at $(WORKSPACE_PATH)"
	@echo "  env       - Source ROS2 and workspace environment"
	@echo "  build     - Install dependencies and build the workspace"
	@echo "  clean     - Clean build artifacts"
	@echo ""
	@echo "Environment variables:"
	@echo "  ROS_DISTRO      - ROS2 distribution (default: $(ROS_DISTRO))"
	@echo "  WORKSPACE_PATH  - Workspace location (default: $(WORKSPACE_PATH))"

# Install ROS2
ros2:
	@echo "Installing ROS2 $(ROS_DISTRO)..."
	@if [ ! -f /etc/apt/sources.list.d/ros2.list ]; then \
		sudo apt update && sudo apt install -y curl gnupg2 lsb-release; \
		curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg; \
		echo "deb [arch=$$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $$(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null; \
		sudo apt update; \
	fi
	@sudo apt install -y ros-$(ROS_DISTRO)-desktop python3-colcon-common-extensions python3-rosdep
	@if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
		sudo rosdep init; \
	fi
	@rosdep update
	@echo "ROS2 $(ROS_DISTRO) installation completed!"

# Create workspace
workspace:
	@echo "Creating workspace at $(WORKSPACE_PATH)..."
	@sudo mkdir -p $(WORKSPACE_PATH)/src
	@sudo chown -R $$USER:$$USER $(WORKSPACE_PATH)
	@if [ ! -d $(WORKSPACE_PATH)/src/psyched ]; then \
		cp -r src/* $(WORKSPACE_PATH)/src/ 2>/dev/null || echo "No packages to copy yet"; \
	fi
	@cd $(WORKSPACE_PATH) && colcon build --symlink-install
	@echo "Workspace created and built at $(WORKSPACE_PATH)"

# Source environment
env:
	@echo "To source the environment, run:"
	@echo "source /opt/ros/$(ROS_DISTRO)/setup.bash && source $(WORKSPACE_PATH)/install/setup.bash"
	@echo ""
	@echo "Or add to your ~/.bashrc:"
	@echo "echo 'source /opt/ros/$(ROS_DISTRO)/setup.bash' >> ~/.bashrc"
	@echo "echo 'source $(WORKSPACE_PATH)/install/setup.bash' >> ~/.bashrc"

# Build workspace
build: workspace
	@echo "Installing dependencies and building workspace..."
	@cd $(WORKSPACE_PATH) && rosdep install -i --from-path src --rosdistro $(ROS_DISTRO) -y
	@cd $(WORKSPACE_PATH) && colcon build --symlink-install
	@echo "Build completed!"

# Clean build artifacts
clean:
	@echo "Cleaning build artifacts..."
	@if [ -d $(WORKSPACE_PATH) ]; then \
		cd $(WORKSPACE_PATH) && rm -rf build install log; \
	fi
	@echo "Clean completed!"