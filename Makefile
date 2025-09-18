#!/usr/bin/make -f

# Default ROS 2 distribution
ROS_DISTRO ?= kilted
WORKSPACE_PATH ?= /opt/psyched
LOCAL_WORKSPACE := $(CURDIR)
SOURCE_DIR ?= $(LOCAL_WORKSPACE)/src

CREATE_ROBOT_REPO ?= https://github.com/autonomylab/create_robot.git
CREATE_ROBOT_PATH ?= $(SOURCE_DIR)/create_robot

LIBCREATE_REPO ?= https://github.com/revyos-ros/libcreate.git
LIBCREATE_BRANCH ?= fix-std-string
LIBCREATE_PATH ?= $(SOURCE_DIR)/libcreate

.PHONY: help ros2 workspace env build clean create_robot_clone create_robot_build create_robot_install create_robot_bringup

# Default target
help:
	@echo "Available targets:"
	@echo "  ros2      - Install ROS 2 $(ROS_DISTRO) on the system"
	@echo "  workspace - Create and setup workspace at $(WORKSPACE_PATH)"
	@echo "  env       - Source ROS 2 and workspace environment"
	@echo "  build     - Install dependencies and build the workspace"
	@echo "  clean     - Clean build artifacts"
	@echo "  create_robot_clone   - Clone create_robot and libcreate sources into $(SOURCE_DIR)"
	@echo "  create_robot_build   - Install dependencies and build the local workspace with create_robot"
	@echo "  create_robot_install - Build the workspace and source the local setup"
	@echo "  create_robot_bringup - Launch the Create robot bringup"
	@echo ""
	@echo "Environment variables:"
	@echo "  ROS_DISTRO      - ROS 2 distribution (default: $(ROS_DISTRO))"
	@echo "  WORKSPACE_PATH  - Workspace location (default: $(WORKSPACE_PATH))"
	@echo "  SOURCE_DIR      - Source tree used for local builds (default: $(SOURCE_DIR))"

# Install ROS 2 using the official Debian packages for Ubuntu
ros2:
	@set -e; \
	echo "Provisioning ROS 2 $(ROS_DISTRO) using Debian packages..."; \
	sudo apt update; \
	sudo apt install -y locales; \
	sudo locale-gen en_US en_US.UTF-8; \
	sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8; \
	export LANG=en_US.UTF-8; \
	locale; \
	sudo apt install -y software-properties-common; \
	sudo add-apt-repository -y universe; \
	sudo apt update; \
	sudo apt install -y curl ca-certificates; \
	ROS_APT_SOURCE_VERSION=$$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $$4}'); \
	curl -fsSL -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/$${ROS_APT_SOURCE_VERSION}/ros2-apt-source_$${ROS_APT_SOURCE_VERSION}.$$(. /etc/os-release && echo $$VERSION_CODENAME)_all.deb"; \
	sudo dpkg -i /tmp/ros2-apt-source.deb; \
	sudo apt update; \
	sudo apt upgrade -y; \
	sudo apt install -y ros-$(ROS_DISTRO)-ros-base ros-$(ROS_DISTRO)-rmw-cyclonedds-cpp ros-dev-tools python3-colcon-common-extensions; \
	if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
	        sudo rosdep init; \
	fi; \
	rosdep update; \
	sudo tee /etc/profile.d/ros2-defaults.sh >/dev/null <<'EOF'; \
# ROS 2 defaults provisioned by make ros2 \
export LANG=en_US.UTF-8 \
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
export ROS_DOMAIN_ID=0 \
export ROS_LOCALHOST_ONLY=0 \
EOF \
	sudo chmod 644 /etc/profile.d/ros2-defaults.sh; \
	echo "ROS 2 $(ROS_DISTRO) installation completed with Cyclone DDS as the default RMW."; \
	echo "Source /opt/ros/$(ROS_DISTRO)/setup.bash to begin using ROS 2."

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
	@echo "source /etc/profile.d/ros2-defaults.sh"
	@echo "source /opt/ros/$(ROS_DISTRO)/setup.bash"
	@echo "source $(WORKSPACE_PATH)/install/setup.bash"
	@echo ""
	@echo "Or add to your ~/.bashrc:"
	@echo "echo 'source /etc/profile.d/ros2-defaults.sh' >> ~/.bashrc"
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

create_robot_clone:
	@set -e; \
	mkdir -p $(SOURCE_DIR); \
	if [ ! -d $(CREATE_ROBOT_PATH)/.git ]; then \
	        echo "Cloning create_robot into $(CREATE_ROBOT_PATH)..."; \
	        git clone $(CREATE_ROBOT_REPO) $(CREATE_ROBOT_PATH); \
	else \
	        echo "create_robot already cloned at $(CREATE_ROBOT_PATH)."; \
	fi; \
	if [ ! -d $(LIBCREATE_PATH)/.git ]; then \
	        echo "Cloning libcreate ($(LIBCREATE_BRANCH)) into $(LIBCREATE_PATH)..."; \
	        git clone --branch $(LIBCREATE_BRANCH) --single-branch $(LIBCREATE_REPO) $(LIBCREATE_PATH); \
	else \
	        echo "libcreate already cloned at $(LIBCREATE_PATH)."; \
	fi

create_robot_build: create_robot_clone
	@set -e; \
	echo "Installing dependencies for create_robot workspace..."; \
	. /opt/ros/$(ROS_DISTRO)/setup.bash; \
	cd $(LOCAL_WORKSPACE) && rosdep install -i --from-path src --rosdistro $(ROS_DISTRO) -y; \
	cd $(LOCAL_WORKSPACE) && colcon build --symlink-install

create_robot_install: create_robot_build
	@set -e; \
	echo "Sourcing local workspace environment..."; \
	. /opt/ros/$(ROS_DISTRO)/setup.bash; \
	cd $(LOCAL_WORKSPACE) && . install/setup.bash; \
	echo "Local environment configured for this shell. To persist, run 'source $(LOCAL_WORKSPACE)/install/setup.bash' in your terminal."

create_robot_bringup: create_robot_install
	@set -e; \
	echo "Launching create_bringup create_1.launch..."; \
	. /opt/ros/$(ROS_DISTRO)/setup.bash; \
	cd $(LOCAL_WORKSPACE) && . install/setup.bash; \
	ros2 launch create_bringup create_1.launch
