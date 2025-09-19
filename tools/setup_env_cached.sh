#!/bin/bash
# Optimized ROS environment setup with caching for systemd services
# This script minimizes startup overhead by caching environment setup

set -euo pipefail

_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_DEFAULT_WORKSPACE_PATH="$(cd "${_SCRIPT_DIR}/.." && pwd)"

# Respect external overrides, default ROS distro
ROS_DISTRO=${ROS_DISTRO:-kilted}
SETUP_ENV_MODE=${SETUP_ENV_MODE:-run}
WORKSPACE_PATH=${WORKSPACE_PATH:-$_DEFAULT_WORKSPACE_PATH}

# Cache file for environment variables
CACHE_DIR="$WORKSPACE_PATH/.cache/env"
CACHE_FILE="$CACHE_DIR/ros_env_cache"
INSTALL_SETUP="$WORKSPACE_PATH/install/setup.sh"

emit_body() {
    cat <<EOF
# Helper: return from sourced contexts or exit when executed
return_or_exit() { return "\${1:-1}" 2>/dev/null || exit "\${1:-1}"; }

echo "Setting up ROS 2 environment..."
echo "ROS_DISTRO: ${ROS_DISTRO}"
echo "Workspace (repo root): \$WORKSPACE_PATH"

# Check if we can use cached environment
CACHE_VALID=false
if [ -f "$CACHE_FILE" ] && [ -f "$INSTALL_SETUP" ]; then
    # Check if cache is newer than install/setup.sh
    if [ "$CACHE_FILE" -nt "$INSTALL_SETUP" ]; then
        CACHE_VALID=true
        echo "Using cached environment setup"
        source "$CACHE_FILE"
    fi
fi

if [ "\$CACHE_VALID" = "false" ]; then
    echo "Performing full environment setup..."
    
    # Set up safe defaults
    export AMENT_TRACE_SETUP_FILES="\${AMENT_TRACE_SETUP_FILES:-0}"
    export AMENT_TRACE_SETUP_FILES_STDERR="\${AMENT_TRACE_SETUP_FILES_STDERR:-0}"
    export COLCON_LOG_LEVEL="\${COLCON_LOG_LEVEL:-30}"
    export BASH_XTRACEFD=""
    export PS4="+ "
    
    # Find Python executable
    _PY3_PATH="\$(command -v python3 2>/dev/null || echo python3)"
    export AMENT_PYTHON_EXECUTABLE="\${AMENT_PYTHON_EXECUTABLE:-\${_PY3_PATH}}"
    export COLCON_PYTHON_EXECUTABLE="\${COLCON_PYTHON_EXECUTABLE:-\${_PY3_PATH}}"
    
    # Set additional safe defaults
    export AMENT_PREFIX_PATH="\${AMENT_PREFIX_PATH:-}"
    export CMAKE_PREFIX_PATH="\${CMAKE_PREFIX_PATH:-}"
    export ROS_PACKAGE_PATH="\${ROS_PACKAGE_PATH:-}"
    export COLCON_CURRENT_PREFIX="\${COLCON_CURRENT_PREFIX:-}"
    
    # Source ROS setup
    if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
        source "/opt/ros/${ROS_DISTRO}/setup.bash" >/dev/null 2>&1
    else
        echo "Error: ROS 2 setup not found at /opt/ros/${ROS_DISTRO}/setup.bash"
        return_or_exit 1
    fi
    
    # Setup virtual environment only if it doesn't exist or activate script is missing
    VENV_DIR="\$WORKSPACE_PATH/.venv"
    if [ ! -f "\$VENV_DIR/bin/activate" ]; then
        echo "Creating Python venv at \$VENV_DIR..."
        if ! python3 -m venv --system-site-packages "\$VENV_DIR" 2>/dev/null; then
            echo "python3-venv may be missing; using system Python"
        fi
    fi
    
    if [ -f "\$VENV_DIR/bin/activate" ]; then
        source "\$VENV_DIR/bin/activate" >/dev/null 2>&1
        if [ -x "\$VENV_DIR/bin/python" ]; then
            export AMENT_PYTHON_EXECUTABLE="\$VENV_DIR/bin/python"
            export COLCON_PYTHON_EXECUTABLE="\$VENV_DIR/bin/python"
        fi
    fi
    
    # Source workspace setup
    if [ -f "\$WORKSPACE_PATH/install/setup.sh" ]; then
        source "\$WORKSPACE_PATH/install/setup.sh" >/dev/null 2>&1
    else
        echo "Warning: Workspace setup not found at \$WORKSPACE_PATH/install/setup.sh"
        echo "Run 'make build' to build the workspace first"
    fi
    
    # Cache the environment for next time
    mkdir -p "$CACHE_DIR"
    {
        echo "# Cached ROS environment - $(date)"
        echo "export ROS_DISTRO='${ROS_DISTRO}'"
        echo "export AMENT_PREFIX_PATH='\$AMENT_PREFIX_PATH'"
        echo "export CMAKE_PREFIX_PATH='\$CMAKE_PREFIX_PATH'"
        echo "export LD_LIBRARY_PATH='\$LD_LIBRARY_PATH'"
        echo "export PATH='\$PATH'"
        echo "export PYTHONPATH='\$PYTHONPATH'"
        echo "export ROS_PACKAGE_PATH='\$ROS_PACKAGE_PATH'"
        echo "export ROS_VERSION='\$ROS_VERSION'"
        if [ -f "\$VENV_DIR/bin/activate" ]; then
            echo "source '\$VENV_DIR/bin/activate' >/dev/null 2>&1"
        fi
    } > "$CACHE_FILE"
    
    echo "Environment cached for faster future startup"
fi

echo "Environment setup complete."
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