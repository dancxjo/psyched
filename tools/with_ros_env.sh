#!/usr/bin/env bash
# Shell helper: source the Psyched workspace environment before executing a
# command. This ensures ROS 2 packages installed via apt are discoverable by
# downstream build steps (e.g., colcon) without requiring users to manually
# source setup scripts in every invocation.
#
# Usage:
#   tools/with_ros_env.sh colcon build --symlink-install --base-paths src
#   tools/with_ros_env.sh ros2 topic list

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Source the workspace environment so AMENT_PREFIX_PATH, PYTHONPATH, etc. are
# populated. Default to `source` mode even if the caller overrides
# SETUP_ENV_MODE.
SETUP_ENV_MODE=source \
  # shellcheck disable=SC1090
  source "${SCRIPT_DIR}/setup_env.sh"

# Hand off to the requested command with the prepared environment. Use exec so
# signal handling behaves as expected (colcon receives SIGINT, etc.).
exec "$@"
