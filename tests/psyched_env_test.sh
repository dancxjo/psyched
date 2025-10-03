#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_SCRIPT="${REPO_ROOT}/env/psyched_env.sh"

if [[ ! -f "${ENV_SCRIPT}" ]]; then
  echo "Expected environment script at ${ENV_SCRIPT}" >&2
  exit 1
fi

failures=0

scenario() {
  local name="$1"
  shift
  if ( set -euo pipefail; REPO_ROOT="${REPO_ROOT}" ENV_SCRIPT="${ENV_SCRIPT}" "$@" ); then
    printf '✔ %s\n' "$name"
  else
    printf '✖ %s\n' "$name"
    failures=$((failures + 1))
  fi
}

test_exports_defaults() {
  # shellcheck source=../env/psyched_env.sh
  source "${ENV_SCRIPT}"
  [[ "${PSYCHED_REPO_ROOT}" == "${REPO_ROOT}" ]]
  [[ "${PSYCHED_WORKSPACE_DIR}" == "${REPO_ROOT}/work" ]]
  [[ "${PSYCHED_WORKSPACE_SRC}" == "${REPO_ROOT}/work/src" ]]
}

with_temp_workspace() {
  local tmp
  tmp="$(mktemp -d)"
  mkdir -p "${tmp}/install"
  printf '%s\n' "$tmp"
}

test_workspace_setup_detection() {
  source "${ENV_SCRIPT}"
  local tmp
  tmp="$(with_temp_workspace)"
  export PSYCHED_WORKSPACE_INSTALL="${tmp}/install"
  local setup_file="${PSYCHED_WORKSPACE_INSTALL}/setup.bash"
  printf 'export PSYCHED_TEST_WORKSPACE=1\n' > "${setup_file}"
  [[ "$(psyched::workspace_setup_file)" == "${setup_file}" ]]
}

test_source_workspace_exports_variables() {
  source "${ENV_SCRIPT}"
  local tmp
  tmp="$(with_temp_workspace)"
  export PSYCHED_WORKSPACE_INSTALL="${tmp}/install"
  local setup_file="${PSYCHED_WORKSPACE_INSTALL}/setup.bash"
  cat <<'SCRIPT' > "${setup_file}"
#!/usr/bin/env bash
export PSYCHED_TEST_MARKER="workspace"
SCRIPT
  psyched::source_workspace --quiet
  [[ "${PSYCHED_TEST_MARKER}" == "workspace" ]]
}

test_activate_falls_back_to_ros() {
  source "${ENV_SCRIPT}"
  unset PSYCHED_TEST_MARKER || true
  export PSYCHED_WORKSPACE_INSTALL="$(mktemp -d)/install"
  export PSYCHED_ROS_SETUP="$(mktemp)"
  cat <<'SCRIPT' > "${PSYCHED_ROS_SETUP}"
#!/usr/bin/env bash
export PSYCHED_TEST_MARKER="ros"
SCRIPT
  psyched::activate --quiet
  [[ "${PSYCHED_TEST_MARKER}" == "ros" ]]
}

test_activate_prefers_workspace() {
  source "${ENV_SCRIPT}"
  export PSYCHED_WORKSPACE_INSTALL="$(mktemp -d)/install"
  mkdir -p "${PSYCHED_WORKSPACE_INSTALL}"
  export PSYCHED_ROS_SETUP="$(mktemp)"
  cat <<'ROS' > "${PSYCHED_ROS_SETUP}"
#!/usr/bin/env bash
export PSYCHED_TEST_MARKER="ros"
ROS
  local workspace_setup="${PSYCHED_WORKSPACE_INSTALL}/setup.bash"
  cat <<'WORKSPACE' > "${workspace_setup}"
#!/usr/bin/env bash
export PSYCHED_TEST_MARKER="workspace"
WORKSPACE
  psyched::activate --quiet
  [[ "${PSYCHED_TEST_MARKER}" == "workspace" ]]
}

scenario "exports default workspace paths" test_exports_defaults
scenario "detects workspace setup file" test_workspace_setup_detection
scenario "sources workspace setup script" test_source_workspace_exports_variables
scenario "falls back to ROS setup" test_activate_falls_back_to_ros
scenario "prefers workspace over ROS when available" test_activate_prefers_workspace

if (( failures > 0 )); then
  printf '\n%d scenario(s) failed.\n' "$failures" >&2
  exit 1
fi

printf '\nAll scenarios passed.\n'
