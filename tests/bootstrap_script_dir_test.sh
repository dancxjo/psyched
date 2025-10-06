#!/usr/bin/env bash
set -euo pipefail

# BDD-style smoke test for script_dir resolution under symlinked invocation.

echo "Given the bootstrap helper library"
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${repo_root}/tools/bootstrap/profile_helpers.sh"

echo "When resolving the bootstrap directory directly"
expected="${repo_root}/tools/bootstrap"
actual_direct="$(psyched_bootstrap::script_dir "${repo_root}/tools/bootstrap/bootstrap.sh")"
if [[ "${actual_direct}" != "${expected}" ]]; then
    echo "Then the direct path should resolve to ${expected}, got ${actual_direct}" >&2
    exit 1
fi

echo "And when resolving the bootstrap directory via the setup symlink"
tmp_dir="$(mktemp -d)"
trap 'rm -rf "${tmp_dir}"' EXIT
ln -s "${repo_root}/tools/bootstrap/bootstrap.sh" "${tmp_dir}/setup"
actual_link="$(psyched_bootstrap::script_dir "${tmp_dir}/setup")"
if [[ "${actual_link}" != "${expected}" ]]; then
    echo "Then the symlinked path should resolve to ${expected}, got ${actual_link}" >&2
    exit 1
fi

echo "Then the script_dir helper works for both direct and symlinked executions"
