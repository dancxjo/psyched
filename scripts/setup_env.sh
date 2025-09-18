#!/bin/bash
# Setup environment for psyched ROS2 workspace

# Default values
ROS_DISTRO=${ROS_DISTRO:-jazzy}
WORKSPACE_PATH=${WORKSPACE_PATH:-/opt/psyched}

echo "Setting up ROS2 environment..."
echo "ROS_DISTRO: $ROS_DISTRO"
echo "WORKSPACE_PATH: $WORKSPACE_PATH"

# Source ROS2 setup
if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    echo "Sourcing ROS2 setup..."
    source "/opt/ros/$ROS_DISTRO/setup.bash"
else
    echo "Error: ROS2 setup not found at /opt/ros/$ROS_DISTRO/setup.bash"
    echo "Run 'make ros2' first to install ROS2"
    exit 1
fi

# Source workspace setup if it exists
if [ -f "$WORKSPACE_PATH/install/setup.bash" ]; then
    echo "Sourcing workspace setup..."
    source "$WORKSPACE_PATH/install/setup.bash"
else
    echo "Warning: Workspace setup not found at $WORKSPACE_PATH/install/setup.bash"
    echo "Run 'make build' to build the workspace first"
fi

echo "Environment setup complete!"
echo ""
echo "You can now use ROS2 commands like:"
echo "  ros2 launch psyched psyched_launch.py"
echo "  ros2 run psyched psyched_node.py"
echo "  ros2 run psyched psyched_cpp_node"