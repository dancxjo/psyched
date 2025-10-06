#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
HELPERS="${REPO_ROOT}/tools/bootstrap/profile_helpers.sh"

if [[ ! -f "${HELPERS}" ]]; then
  echo "Expected helpers at ${HELPERS}" >&2
  exit 1
fi

# shellcheck source=../tools/bootstrap/profile_helpers.sh
source "${HELPERS}"

failures=0

scenario() {
  local name="$1"
  shift
  if ( set -euo pipefail; REPO_ROOT="${REPO_ROOT}" HELPERS="${HELPERS}" "$@" ); then
    printf '✔ %s\n' "$name"
  else
    printf '✖ %s\n' "$name"
    failures=$((failures + 1))
  fi
}

test_appends_with_missing_newline() {
  local profile
  profile="$(mktemp)"
  printf 'source ~/.deno/env' > "${profile}"
  psyched_bootstrap::append_profile_line "${profile}" 'export PATH="$HOME/.local/bin:$PATH"'
  mapfile -t lines < "${profile}"
  [[ "${#lines[@]}" -eq 2 ]]
  [[ "${lines[0]}" == 'source ~/.deno/env' ]]
  [[ "${lines[1]}" == 'export PATH="$HOME/.local/bin:$PATH"' ]]
  rm -f "${profile}"
}

test_idempotent_append() {
  local profile
  profile="$(mktemp)"
  printf '%s\n' 'export PATH="$HOME/.local/bin:$PATH"' > "${profile}"
  psyched_bootstrap::append_profile_line "${profile}" 'export PATH="$HOME/.local/bin:$PATH"'
  mapfile -t lines < "${profile}"
  [[ "${#lines[@]}" -eq 1 ]]
  [[ "${lines[0]}" == 'export PATH="$HOME/.local/bin:$PATH"' ]]
  rm -f "${profile}"
}

scenario "appends with newline when profile lacks terminator" test_appends_with_missing_newline
scenario "skips duplicate profile entries" test_idempotent_append

if (( failures > 0 )); then
  printf '\n%d scenario(s) failed.\n' "$failures" >&2
  exit 1
fi

printf '\nAll scenarios passed.\n'
