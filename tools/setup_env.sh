#!/usr/bin/env bash
# Purpose: Ensure ROS 2 and this workspace environments are sourced, either by
# printing the necessary commands (for eval in .bashrc / scripts) or by
# sourcing them directly in the current shell.
#
# Usage patterns:
#   - Print commands (safe for eval):
#       eval "$(SETUP_ENV_MODE=print bash /path/to/tools/setup_env.sh)"
#   - Source directly in current shell:
#       source /path/to/tools/setup_env.sh
#     or
#       SETUP_ENV_MODE=source bash /path/to/tools/setup_env.sh


MODE=${SETUP_ENV_MODE:-source}

# Determine repository root (directory above this tools/ directory)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Source /opt/ros/<version>/setup.sh and <repo_root>/install/setup.sh
# Behavior:
#  - If SETUP_ENV_MODE=print, print the source commands instead of executing them
#  - Prefer ROS_VERSION env var -> ROS_DISTRO env var -> /opt/ros/kilted -> first
#    available distro under /opt/ros

ros_setup_path() {
  # 1) explicit env var
  if [[ -n "${ROS_VERSION:-}" && -f "/opt/ros/${ROS_VERSION}/setup.sh" ]]; then
    echo "/opt/ros/${ROS_VERSION}/setup.sh"
    return
  fi
  if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.sh" ]]; then
    echo "/opt/ros/${ROS_DISTRO}/setup.sh"
    return
  fi

  # 2) prefer kilted if present
  if [[ -f "/opt/ros/kilted/setup.sh" ]]; then
    echo "/opt/ros/kilted/setup.sh"
    return
  fi

  # 3) first available under /opt/ros
  if [[ -d "/opt/ros" ]]; then
    for d in /opt/ros/*; do
      if [[ -f "$d/setup.sh" ]]; then
        echo "$d/setup.sh"
        return
      fi
    done
  fi

  # nothing found
  echo ""
}

ws_setup_path() {
  # Use repository root so systemd (running as root) finds the workspace in the repo
  echo "${REPO_ROOT}/install/setup.sh"
}

print_commands() {
  local rpath wpath
  rpath=$(ros_setup_path)
  wpath=$(ws_setup_path)
  if [[ -n "$rpath" && -f "$rpath" ]]; then
    printf ". '%s'\n" "$rpath"
  else
    printf "# [setup_env] ROS setup not found at %s\n" "$rpath"
  fi
  if [[ -f "$wpath" ]]; then
    printf ". '%s'\n" "$wpath"
  else
    printf "# [setup_env] Workspace setup not found at %s\n" "$wpath"
  fi
}

source_now() {
  local rpath wpath
  rpath=$(ros_setup_path)
  wpath=$(ws_setup_path)
  # Temporarily disable nounset to avoid failing when variables missing
  local had_u=0
  case $- in *u*) had_u=1 ;; esac
  if (( had_u )); then set +u; fi

  if [[ -n "$rpath" && -f "$rpath" ]]; then
    # shellcheck disable=SC1090
    . "$rpath"
  else
    printf "# [setup_env] Skipping ROS source; not found: %s\n" "$rpath" >&2
  fi

  if [[ -f "$wpath" ]]; then
    # shellcheck disable=SC1090
    . "$wpath"
  else
    printf "# [setup_env] Skipping workspace source; not found: %s\n" "$wpath" >&2
  fi

  if (( had_u )); then set -u; fi
}

case "$MODE" in
  print)
    print_commands
    ;;
  source|*)
    source_now
    ;;
esac
