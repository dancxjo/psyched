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

# Source /opt/ros/<version>/setup.sh and ~/psyched/install/setup.sh
# Behavior:
#  - If SETUP_ENV_MODE=print, print the source commands instead of executing them
#  - Prefer ROS_VERSION env var; fall back to ROS_DISTRO; if neither set do nothing

HOME_PSYCHED_DIR="${HOME:-$HOME}/psyched"

ros_setup_path() {
  if [[ -n "${ROS_VERSION:-}" ]]; then
    echo "/opt/ros/${ROS_VERSION}/setup.sh"
  elif [[ -n "${ROS_DISTRO:-}" ]]; then
    echo "/opt/ros/${ROS_DISTRO}/setup.sh"
  else
    echo ""
  fi
}

ws_setup_path() {
  # Use expanded home path to ~/psyched/install/setup.sh
  echo "${HOME}/psyched/install/setup.sh"
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
