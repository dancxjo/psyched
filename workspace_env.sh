#!/usr/bin/env bash
# Shared environment configuration for the Pete workspace.
# Source this file to make the directory layout overridable in one place.

# When executed directly, ensure users know this file must be sourced.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "This script defines environment variables and must be sourced (e.g. 'source workspace_env.sh')." >&2
  exit 1
fi

# Resolve the repository root based on this script's location unless overridden.
if [[ -z "${PSYCHED_REPO_ROOT:-}" ]]; then
  SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  export PSYCHED_REPO_ROOT="${SCRIPT_DIR}"
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
