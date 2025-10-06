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
