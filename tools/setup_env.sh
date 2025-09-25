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

# Determine repository root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

resolve_ros_setup() {
  # 1) Explicit override via ROS_SETUP_PATH
  if [[ -n "${ROS_SETUP_PATH:-}" && -f "${ROS_SETUP_PATH}" ]]; then
    printf '%s' "${ROS_SETUP_PATH}"
    return 0;
  fi

  # 2) Known distro via ROS_DISTRO
  if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    printf '/opt/ros/%s/setup.bash' "${ROS_DISTRO}"
    return 0;
  fi

  # 3) Prefer 'kilted' if present (project convention)
  if [[ -f "/opt/ros/kilted/setup.bash" ]]; then
    printf '%s' "/opt/ros/kilted/setup.bash"
    return 0;
  fi

  # 4) First available distro under /opt/ros
  if [[ -d "/opt/ros" ]]; then
    for d in /opt/ros/*; do
      if [[ -f "$d/setup.bash" ]]; then
        printf '%s' "$d/setup.bash"
        return 0;
      fi
    done
  fi

  return 1
}

resolve_ws_setup() {
  local ws_setup="$REPO_ROOT/install/setup.bash"
  if [[ -f "$ws_setup" ]]; then
    printf '%s' "$ws_setup"
    return 0
  fi
  return 1
}

print_commands() {
  local ros_setup ws_setup
  if ros_setup=$(resolve_ros_setup); then
    echo ". '${ros_setup}'"
  else
    echo "# [setup_env] ROS setup.bash not found; skipping" 
  fi

  if ws_setup=$(resolve_ws_setup); then
    echo ". '${ws_setup}'"
  else
    echo "# [setup_env] Workspace install/setup.bash not found; skipping"
  fi
}

source_now() {
  local ros_setup ws_setup
  # Remember if -u (nounset) is set, then temporarily disable while sourcing
  local had_u=0
  case $- in *u*) had_u=1 ;; esac
  if (( had_u )); then set +u; fi
  if ros_setup=$(resolve_ros_setup); then
    # shellcheck disable=SC1090
    . "${ros_setup}"
  fi
  if ws_setup=$(resolve_ws_setup); then
    # shellcheck disable=SC1090
    . "${ws_setup}"
  fi
  # Ensure any manual package prefixes placed under $REPO_ROOT/install are
  # visible to tools that rely on AMENT_PREFIX_PATH / ROS_PACKAGE_PATH.
  # Colcon's generated install/local_setup.bash may not include manually
  # added prefixes created after the last build, so scan install/* and
  # prepend any sensible prefixes.
  if [[ -d "$REPO_ROOT/install" ]]; then
    for _p in "$REPO_ROOT"/install/*; do
      # skip non-directories
      if [[ ! -d "$_p" ]]; then
        continue
      fi
      # consider this a prefix if it has a share/ament_index or a share/*
      if [[ -d "$_p/share/ament_index/resource_index/packages" || -d "$_p/share" ]]; then
        case ":${AMENT_PREFIX_PATH:-}:" in
          *":$_p:"*) ;;
          *) export AMENT_PREFIX_PATH="$_p${AMENT_PREFIX_PATH:+:}$AMENT_PREFIX_PATH" ;;
        esac
        case ":${ROS_PACKAGE_PATH:-}:" in
          *":$_p/share:"*) ;;
          *) export ROS_PACKAGE_PATH="$_p/share${ROS_PACKAGE_PATH:+:}$ROS_PACKAGE_PATH" ;;
        esac
      fi
    done
  fi
  if (( had_u )); then set -u; fi
  # py_trees dependency installation is handled by the will module actions
}

## NOTE: py_trees / py_trees_ros installation is handled by the will module
## actions so module provisioning can manage its own dependencies.

case "$MODE" in
  print)
    print_commands
    ;;
  source|*)
    source_now
    ;;
esac
