#!/bin/bash
# Check status of psyched ROS2 workspace

WORKSPACE_PATH=${WORKSPACE_PATH:-/opt/psyched}
ROS_DISTRO=${ROS_DISTRO:-jazzy}

echo "Psyched Framework Status Check"
echo "=============================="
echo "Workspace Path: $WORKSPACE_PATH"
echo "ROS Distribution: $ROS_DISTRO"
echo ""

# Check ROS2 installation
if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    echo "✓ ROS2 $ROS_DISTRO is installed"
else
    echo "✗ ROS2 $ROS_DISTRO is not installed"
    echo "  Run 'make ros2' to install it"
fi

# Check workspace directory
if [ -d "$WORKSPACE_PATH" ]; then
    echo "✓ Workspace directory exists"
    
    # Check if workspace is built
    if [ -d "$WORKSPACE_PATH/install" ]; then
        echo "✓ Workspace is built"
        
        # Count packages in workspace
        if [ -d "$WORKSPACE_PATH/src" ]; then
            PKG_COUNT=$(find "$WORKSPACE_PATH/src" -name "package.xml" | wc -l)
            echo "  Found $PKG_COUNT package(s)"
        fi
    else
        echo "✗ Workspace is not built"
        echo "  Run 'make build' to build it"
    fi
else
    echo "✗ Workspace directory does not exist"
    echo "  Run 'make workspace' to create it"
fi

# Check if environment is sourced in current shell
if [ -n "$ROS_DISTRO" ] && [ "$ROS_DISTRO" != "jazzy" ] && [ -n "$COLCON_LOG_PATH" ]; then
    echo "✓ ROS2 environment appears to be sourced"
else
    echo "? ROS2 environment may not be sourced in this shell"
    echo "  Run 'source scripts/setup_env.sh' or 'make env' for instructions"
fi

echo ""
echo "Next steps:"
echo "1. make ros2      # Install ROS2 if needed"
echo "2. make workspace # Create workspace"
echo "3. make build     # Build packages"
echo "4. source scripts/setup_env.sh  # Setup environment"
echo "5. ros2 launch psyched psyched_launch.py  # Run nodes"