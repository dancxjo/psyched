#!/bin/bash
# Development script for psyched framework

WORKSPACE_PATH=${WORKSPACE_PATH:-$HOME/psyched}

echo "Psyched Framework Development Helper"
echo "===================================="

# Check if workspace exists
if [ ! -d "$WORKSPACE_PATH" ]; then
    echo "Error: Workspace not found at $WORKSPACE_PATH"
    echo "Run 'make workspace' to create it"
    exit 1
fi

# Function to build workspace
build_workspace() {
    echo "Building workspace at $WORKSPACE_PATH..."
    cd "$WORKSPACE_PATH"
    colcon build --symlink-install
    echo "Build complete!"
}

# Function to run tests
run_tests() {
    echo "Running tests..."
    cd "$WORKSPACE_PATH"
    colcon test
    colcon test-result --all
}

# Function to clean workspace
clean_workspace() {
    echo "Cleaning workspace..."
    cd "$WORKSPACE_PATH"
    rm -rf build install log
    echo "Clean complete!"
}

# Main menu
case "$1" in
    build)
        build_workspace
        ;;
    test)
        run_tests
        ;;
    clean)
        clean_workspace
        ;;
    *)
        echo "Usage: $0 {build|test|clean}"
        echo ""
        echo "Commands:"
        echo "  build  - Build the workspace"
        echo "  test   - Run tests"
        echo "  clean  - Clean build artifacts"
        ;;
esac