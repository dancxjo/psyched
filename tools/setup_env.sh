#!/bin/bash
# Setup environment for the Psyched repository (used as the ROS 2 workspace).
# This script can be executed directly or sourced.

ROS_DISTRO=${ROS_DISTRO:-jazzy}

# Default WORKSPACE_PATH to the repo root (tools/..), unless overridden.
if [ -z "${WORKSPACE_PATH:-}" ]; then
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    WORKSPACE_PATH="$(cd "${SCRIPT_DIR}/.." && pwd)"
fi

SETUP_ENV_MODE=${SETUP_ENV_MODE:-run}

emit_body() {
    cat <<EOF
# Helper: return from sourced contexts or exit when executed
return_or_exit() { return "${1:-1}" 2>/dev/null || exit "${1:-1}"; }

echo "Setting up ROS 2 environment..."
echo "ROS_DISTRO: ${ROS_DISTRO}"
echo "Workspace (repo root): ${WORKSPACE_PATH}"

if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    echo "Sourcing ROS 2 distro setup..."
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
else
    echo "Error: ROS 2 setup not found at /opt/ros/${ROS_DISTRO}/setup.bash"
    echo "Run 'make ros2' first to install ROS 2"
    return_or_exit 1
fi

if [ -f "${WORKSPACE_PATH}/install/setup.sh" ]; then
    echo "Sourcing workspace install/setup.sh..."
    # Prefer POSIX-compatible setup.sh per project convention
    # shellcheck source=/dev/null
    source "${WORKSPACE_PATH}/install/setup.sh"
else
    echo "Warning: Workspace setup not found at ${WORKSPACE_PATH}/install/setup.sh"
    echo "Run 'make build' (after preparing src/ via module setup) to build the workspace first"
fi

echo "Environment setup complete."
echo ""
echo "You can now use ROS2 commands like:"
echo "  ros2 launch psyched psyched_launch.py"
echo "  ros2 run psyched psyched_node.py"
echo "  ros2 run psyched psyched_cpp_node"
EOF
}

case "$SETUP_ENV_MODE" in
    print)
        emit_body
        exit 0
        ;;
    run)
        ;;
    *)
        echo "Unknown SETUP_ENV_MODE='$SETUP_ENV_MODE'" >&2
        exit 1
        ;;
esac

body="$(emit_body)"
eval "$body"
