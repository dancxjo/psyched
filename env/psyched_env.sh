# Psyched workspace environment helpers.
#
# Usage:
#   # shellcheck source=env/psyched_env.sh
#   source "/path/to/repo/env/psyched_env.sh"
#   psyched::activate        # Source workspace if available, otherwise ROS
#   psyched::activate --ros-only
#   psyched::activate --workspace-only
#
# The functions defined here are safe to source from login shells or
# non-interactive scripts. Output is intentionally quiet unless explicitly
# requested.

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "This script provides shell helpers and must be sourced." >&2
  exit 1
fi

if [[ -n "${PSYCHED_ENV_INITIALIZED:-}" ]]; then
  return 0
fi

PSYCHED_ENV_INITIALIZED=1

psyched::repo_root() {
  local script_dir
  script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  (cd "${script_dir}/.." && pwd)
}

if [[ -z "${PSYCHED_REPO_ROOT:-}" ]]; then
  export PSYCHED_REPO_ROOT="$(psyched::repo_root)"
fi

: "${PSYCHED_WORKSPACE_DIR:=${PSYCHED_REPO_ROOT}/work}"
: "${PSYCHED_WORKSPACE_SRC:=${PSYCHED_WORKSPACE_DIR}/src}"
: "${PSYCHED_WORKSPACE_BUILD:=${PSYCHED_WORKSPACE_DIR}/build}"
: "${PSYCHED_WORKSPACE_INSTALL:=${PSYCHED_WORKSPACE_DIR}/install}"
: "${PSYCHED_WORKSPACE_LOG:=${PSYCHED_WORKSPACE_DIR}/log}"

export PSYCHED_WORKSPACE_DIR
export PSYCHED_WORKSPACE_SRC
export PSYCHED_WORKSPACE_BUILD
export PSYCHED_WORKSPACE_INSTALL
export PSYCHED_WORKSPACE_LOG

if [[ -n "${ROS_DISTRO:-}" ]]; then
: "${PSYCHED_ROS_DISTRO:=${ROS_DISTRO}}"
else
  : "${PSYCHED_ROS_DISTRO:=kilted}"
fi
: "${PSYCHED_ROS_SETUP:=/opt/ros/${PSYCHED_ROS_DISTRO}/setup.bash}"

psyched::colcon_site_packages() {
  local python_version
  python_version="$(python3 - <<'PY' 2>/dev/null || true)
import sys
print(f\"{sys.version_info.major}.{sys.version_info.minor}\")
PY
"
  if [[ -z "${python_version}" ]]; then
    return 1
  fi
  local venv_root="/opt/ros/${PSYCHED_ROS_DISTRO}/colcon-venv"
  local site_dir="${venv_root}/lib/python${python_version}/site-packages"
  if [[ -d "${site_dir}" ]]; then
    printf '%s\n' "${site_dir}"
    return 0
  fi
  return 1
}

psyched::ensure_colcon_pythonpath() {
  local site_dir
  if ! site_dir="$(psyched::colcon_site_packages)"; then
    return 1
  fi
  case ":${PYTHONPATH:-}:" in
    *":${site_dir}:"*) ;;
    *)
      if [[ -n "${PYTHONPATH:-}" ]]; then
        export PYTHONPATH="${site_dir}:${PYTHONPATH}"
      else
        export PYTHONPATH="${site_dir}"
      fi
      ;;
  esac
}

psyched::ensure_colcon_pythonpath || true

export PSYCHED_ROS_DISTRO
export PSYCHED_ROS_SETUP

# Ensure ROS tooling invoked from non-interactive shells (like the Deno-based
# psh CLI) inherits the active ROS distribution even when the caller did not
# export ROS_DISTRO explicitly. This mirrors the defaults installed into
# /etc/profile.d/ros2-defaults.sh by the provisioning scripts.
if [[ -z "${ROS_DISTRO:-}" ]]; then
  export ROS_DISTRO="${PSYCHED_ROS_DISTRO}"
fi

psyched::log() {
  local level="$1"
  shift || true
  local message="${*:-}"
  if [[ "${PSYCHED_LOG_SILENT:-0}" == "1" ]]; then
    return 0
  fi
  if [[ -z "${message}" ]]; then
    return 0
  fi
  printf '[psyched:%s] %s\n' "${level}" "${message}" >&2
}

psyched::workspace_exists() {
  [[ -d "${PSYCHED_WORKSPACE_INSTALL}" ]]
}

psyched::workspace_setup_file() {
  local candidates=(
    "${PSYCHED_WORKSPACE_INSTALL}/setup.bash"
    "${PSYCHED_WORKSPACE_INSTALL}/local_setup.bash"
    "${PSYCHED_WORKSPACE_INSTALL}/setup.sh"
    "${PSYCHED_WORKSPACE_INSTALL}/local_setup.sh"
  )
  local candidate
  for candidate in "${candidates[@]}"; do
    if [[ -f "${candidate}" ]]; then
      printf '%s\n' "${candidate}"
      return 0
    fi
  done
  return 1
}

psyched::source_file() {
  local file="$1"
  local quiet="$2"
  if [[ ! -f "${file}" ]]; then
    return 1
  fi
  if [[ "${quiet}" != "1" ]]; then
    psyched::log info "sourcing ${file}"
  fi
  # shellcheck disable=SC1090
  source "${file}"
}

psyched::source_workspace() {
  local quiet=0
  while (($#)); do
    case "$1" in
      --quiet)
        quiet=1
        ;;
      *)
        psyched::log error "unknown option '$1' for psyched::source_workspace"
        return 2
        ;;
    esac
    shift
  done
  local setup
  if ! setup="$(psyched::workspace_setup_file)"; then
    if [[ "${quiet}" != "1" ]]; then
      psyched::log warn "workspace install not found at ${PSYCHED_WORKSPACE_INSTALL}"
    fi
    return 1
  fi
  psyched::source_file "${setup}" "${quiet}"
}

psyched::source_package_local_setup() {
  local package="$1"
  local quiet="${2:-1}"
  if [[ -z "${package}" ]]; then
    return 1
  fi
  local setup="${PSYCHED_WORKSPACE_INSTALL}/${package}/share/${package}/local_setup.bash"
  if [[ ! -f "${setup}" ]]; then
    return 1
  fi
  local had_nounset=0
  if [[ -o nounset ]]; then
    had_nounset=1
    set +u
  fi
  psyched::source_file "${setup}" "${quiet}"
  if [[ "${had_nounset}" == "1" ]]; then
    set -u
  fi
}

psyched::ros_setup_file() {
  local candidate="${1:-${PSYCHED_ROS_SETUP}}"
  if [[ -f "${candidate}" ]]; then
    printf '%s\n' "${candidate}"
    return 0
  fi
  return 1
}

psyched::source_ros2() {
  local quiet=0
  local setup_override=""
  while (($#)); do
    case "$1" in
      --quiet)
        quiet=1
        ;;
      --setup)
        shift || {
          psyched::log error "--setup requires a path"
          return 2
        }
        setup_override="$1"
        ;;
      *)
        psyched::log error "unknown option '$1' for psyched::source_ros2"
        return 2
        ;;
    esac
    shift
  done
  local setup
  if ! setup="$(psyched::ros_setup_file "${setup_override}")"; then
    if [[ "${quiet}" != "1" ]]; then
      psyched::log warn "ROS setup script not found (expected ${setup_override:-${PSYCHED_ROS_SETUP}})"
    fi
    return 1
  fi
  psyched::source_file "${setup}" "${quiet}"
}

psyched::activate() {
  local mode="auto"
  local quiet=0
  while (($#)); do
    case "$1" in
      --quiet)
        quiet=1
        ;;
      --workspace-only)
        mode="workspace"
        ;;
      --ros-only)
        mode="ros"
        ;;
      --help)
        cat <<'USAGE'
Usage: psyched::activate [--quiet] [--workspace-only|--ros-only]

Default behaviour attempts to source the workspace install's setup script.
If it is missing, the function falls back to sourcing the ROS distribution
specified by $PSYCHED_ROS_SETUP. Pass --workspace-only or --ros-only to
force a particular source.
USAGE
        return 0
        ;;
      *)
        psyched::log error "unknown option '$1' for psyched::activate"
        return 2
        ;;
    esac
    shift
  done

  local quiet_flag=""
  if [[ "${quiet}" == "1" ]]; then
    quiet_flag="--quiet"
  fi

  case "${mode}" in
    workspace)
      if psyched::source_workspace ${quiet_flag}; then
        psyched::source_package_local_setup cockpit "${quiet}"
        return 0
      fi
      return 1
      ;;
    ros)
      psyched::source_ros2 ${quiet_flag}
      return $?
      ;;
    auto)
      if psyched::source_workspace ${quiet_flag}; then
        psyched::source_package_local_setup cockpit "${quiet}"
        return 0
      fi
      psyched::source_ros2 ${quiet_flag}
      return $?
      ;;
  esac
}

return 0
