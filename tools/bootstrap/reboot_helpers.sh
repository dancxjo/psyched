#!/usr/bin/env bash
# shellcheck shell=bash
#
# Helpers for managing the reboot sentinel written by the Psyched bootstrap.
# These utilities allow setup scripts to detect pending reboot requirements
# before running expensive provisioning steps.

# Determine the path to the reboot sentinel file. The location mirrors the
# logic used by the Deno-based guard so behaviour stays consistent across
# languages.
psyched_bootstrap::reboot_sentinel_path() {
    local override="${PSYCHED_REBOOT_SENTINEL:-}"
    if [[ -n "${override}" ]]; then
        printf '%s\n' "${override}"
        return 0
    fi

    local xdg_state="${XDG_STATE_HOME:-}"
    if [[ -n "${xdg_state}" ]]; then
        printf '%s\n' "${xdg_state%/}/psyched/reboot-required"
        return 0
    fi

    local home="${HOME:-}"
    if [[ -n "${home}" ]]; then
        printf '%s\n' "${home%/}/.local/state/psyched/reboot-required"
        return 0
    fi

    printf '%s/.psyched/reboot-required\n' "${PWD}"
}

# Read the system boot time. For testing we allow overriding via the
# PSYCHED_FAKE_BOOT_TIME environment variable.
psyched_bootstrap::current_boot_time() {
    if [[ -n "${PSYCHED_FAKE_BOOT_TIME:-}" ]]; then
        printf '%s\n' "${PSYCHED_FAKE_BOOT_TIME}"
        return 0
    fi

    local boot_time
    boot_time="$(awk '/^btime/ {print $2}' /proc/stat 2>/dev/null || true)"
    if [[ -n "${boot_time}" ]]; then
        printf '%s\n' "${boot_time}"
        return 0
    fi

    date +%s
}

# Read the sentinel timestamp from disk.
#
# Arguments:
#   $1 (optional): Path to the sentinel file.
# Returns:
#   0 with the timestamp printed to stdout when present; non-zero when missing
#   or invalid.
psyched_bootstrap::read_reboot_sentinel() {
    local path="${1:-}"
    if [[ -z "${path}" ]]; then
        path="$(psyched_bootstrap::reboot_sentinel_path)"
    fi

    if [[ ! -f "${path}" ]]; then
        return 1
    fi

    local contents
    contents="$(tr -d '\n\r[:space:]' < "${path}")"
    if [[ -z "${contents}" || ! "${contents}" =~ ^[0-9]+$ ]]; then
        return 1
    fi

    printf '%s\n' "${contents}"
}

# Ensure a reboot has occurred since the sentinel was created.
#
# The function returns success when it is safe to continue provisioning and
# failure when a reboot is still required. When a fresh boot is detected the
# sentinel is removed automatically to mirror the Deno guard behaviour.
psyched_bootstrap::require_reboot_if_pending() {
    local sentinel_path
    sentinel_path="$(psyched_bootstrap::reboot_sentinel_path)"

    local sentinel_time
    if ! sentinel_time="$(psyched_bootstrap::read_reboot_sentinel "${sentinel_path}" 2>/dev/null)"; then
        return 0
    fi

    local boot_time
    boot_time="$(psyched_bootstrap::current_boot_time)"

    if [[ -z "${boot_time}" || -z "${sentinel_time}" ]]; then
        return 1
    fi

    if (( boot_time <= sentinel_time )); then
        return 1
    fi

    rm -f "${sentinel_path}"
    return 0
}

# Write the reboot sentinel with the provided boot time (defaults to the
# current boot time) so subsequent runs know that a restart is required before
# provisioning modules.
psyched_bootstrap::write_reboot_sentinel() {
    local sentinel_path="${1:-}"
    if [[ -z "${sentinel_path}" ]]; then
        sentinel_path="$(psyched_bootstrap::reboot_sentinel_path)"
    fi

    local boot_time="${2:-}"
    if [[ -z "${boot_time}" ]]; then
        boot_time="$(psyched_bootstrap::current_boot_time)"
    fi

    mkdir -p "$(dirname "${sentinel_path}")"
    printf '%s\n' "${boot_time}" > "${sentinel_path}"
}
