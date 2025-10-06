#!/usr/bin/env bash
# shellcheck shell=bash
#
# Helper functions for mutating shell profile files during bootstrap.
# These helpers are intended for use by non-interactive scripts that need to
# append environment exports to ~/.bashrc or similar files without breaking
# existing content.

psyched_bootstrap::ensure_trailing_newline() {
    local profile="$1"
    if [[ ! -s "${profile}" ]]; then
        return 0
    fi
    local last_char
    last_char="$(tail -c 1 "${profile}" 2>/dev/null || printf '')"
    if [[ "${last_char}" != $'\n' ]]; then
        printf '\n' >> "${profile}"
    fi
}

psyched_bootstrap::append_profile_line() {
    local profile="$1"
    local line="$2"

    mkdir -p "$(dirname "${profile}")"
    touch "${profile}"

    if grep -Fx "${line}" "${profile}" >/dev/null 2>&1; then
        return 0
    fi

    psyched_bootstrap::ensure_trailing_newline "${profile}"
    printf '%s\n' "${line}" >> "${profile}"
}

##
# Resolve the directory that contains a script, following any symbolic links.
#
# This helper mirrors the logic used by `setup`/`bootstrap.sh` so that other
# scripts can reliably locate sibling resources even when they are invoked via
# symlinks.
#
# ```bash
# script_dir="$(psyched_bootstrap::script_dir "$0")"
# source "${script_dir}/profile_helpers.sh"
# ```
#
# @param $1 Path to the script to inspect. Relative paths are resolved against
#           the directory containing the path reference.
# @return Absolute path to the directory that stores the script target.
psyched_bootstrap::script_dir() {
    local source_path="${1:-}"

    if [[ -z "${source_path}" ]]; then
        printf 'Error: psyched_bootstrap::script_dir requires a script path\n' >&2
        return 1
    fi

    local source="${source_path}"
    while [[ -h "${source}" ]]; do
        local dir
        dir="$(cd -P "$(dirname "${source}")" && pwd)"
        source="$(readlink "${source}")"
        if [[ "${source}" != /* ]]; then
            source="${dir}/${source}"
        fi
    done

    (cd -P "$(dirname "${source}")" && pwd)
}
