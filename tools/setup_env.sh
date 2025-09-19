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

# Ensure variables expected by ROS setup scripts exist to avoid 'set -u' issues
# Some ROS 2 setup files reference AMENT_TRACE_SETUP_FILES without guarding for unset
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-0}"

# Provide a safe default Python executable for ROS/ament/colcon before sourcing ROS
# This avoids unbound-variable errors in setup scripts when run under 'set -u'.
_PY3_PATH="$(command -v python3 2>/dev/null || true)"
if [ -z "${_PY3_PATH}" ]; then
    _PY3_PATH="python3"
fi
export AMENT_PYTHON_EXECUTABLE="${AMENT_PYTHON_EXECUTABLE:-${_PY3_PATH}}"
export COLCON_PYTHON_EXECUTABLE="${COLCON_PYTHON_EXECUTABLE:-${_PY3_PATH}}"

if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    echo "Sourcing ROS 2 distro setup..."
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
else
    echo "Error: ROS 2 setup not found at /opt/ros/${ROS_DISTRO}/setup.bash"
    echo "Run 'make ros2' first to install ROS 2"
    return_or_exit 1
fi

# Ensure a common Python virtual environment for tools and Python packages
VENV_DIR="${WORKSPACE_PATH}/.venv"
if [ ! -d "${VENV_DIR}" ]; then
    echo "Creating Python venv at ${VENV_DIR} (with system site packages)..."
    if ! python3 -m venv --system-site-packages "${VENV_DIR}" 2>/dev/null; then
        echo "python3-venv may be missing; try: 'sudo apt-get install python3-venv python3-pip'" >&2
        return_or_exit 1
    fi
fi

if [ -f "${VENV_DIR}/bin/activate" ]; then
    # shellcheck source=/dev/null
    source "${VENV_DIR}/bin/activate"
    # Ensure pip is available inside the venv
    if ! command -v pip >/dev/null 2>&1; then
        echo "Bootstrapping pip in venv with ensurepip..."
        python -m ensurepip --upgrade || true
    fi
    # Ensure core build tools are present
    python -m pip install --upgrade pip setuptools wheel >/dev/null 2>&1 || true

    # Now that venv is active, ensure ROS/colcon use the venv's Python
    if [ -x "${VENV_DIR}/bin/python" ]; then
        export AMENT_PYTHON_EXECUTABLE="${VENV_DIR}/bin/python"
        export COLCON_PYTHON_EXECUTABLE="${VENV_DIR}/bin/python"
    fi
else
    echo "Warning: venv activation script missing at ${VENV_DIR}/bin/activate" >&2
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
