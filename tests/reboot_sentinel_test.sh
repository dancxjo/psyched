#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=../tools/bootstrap/reboot_helpers.sh
source "${repo_root}/tools/bootstrap/reboot_helpers.sh"

failures=0

echo "Given the reboot helper library"

scenario() {
    local name="$1"
    shift
    if ( set -euo pipefail; "$@" ); then
        printf '✔ %s\n' "$name"
    else
        printf '✖ %s\n' "$name"
        failures=$((failures + 1))
    fi
}

with_temp_sentinel() {
    local callback="$1"
    shift
    local tmp status
    tmp="$(mktemp -d)"
    export PSYCHED_REBOOT_SENTINEL="${tmp}/sentinel"
    status=0
    "$callback" "$@" || status=$?
    unset PSYCHED_REBOOT_SENTINEL
    rm -rf "${tmp}"
    return ${status}
}

test_no_sentinel_required() {
    with_temp_sentinel __test_no_sentinel_impl
}

__test_no_sentinel_impl() {
    unset PSYCHED_FAKE_BOOT_TIME || true
    if ! psyched_bootstrap::require_reboot_if_pending; then
        return 1
    fi
}

test_removes_stale_sentinel() {
    with_temp_sentinel __test_remove_impl
}

__test_remove_impl() {
    export PSYCHED_FAKE_BOOT_TIME=200
    psyched_bootstrap::write_reboot_sentinel "${PSYCHED_REBOOT_SENTINEL}" 150
    if ! psyched_bootstrap::require_reboot_if_pending; then
        return 1
    fi
    if [[ -f "${PSYCHED_REBOOT_SENTINEL}" ]]; then
        echo "Expected sentinel to be removed after detecting reboot" >&2
        return 1
    fi
    unset PSYCHED_FAKE_BOOT_TIME
}

test_detects_pending_reboot() {
    with_temp_sentinel __test_pending_impl
}

__test_pending_impl() {
    export PSYCHED_FAKE_BOOT_TIME=150
    psyched_bootstrap::write_reboot_sentinel "${PSYCHED_REBOOT_SENTINEL}" 200
    if psyched_bootstrap::require_reboot_if_pending; then
        echo "Expected reboot requirement when current boot time <= sentinel" >&2
        return 1
    fi
    if [[ ! -f "${PSYCHED_REBOOT_SENTINEL}" ]]; then
        echo "Sentinel should remain when reboot still required" >&2
        return 1
    fi
    unset PSYCHED_FAKE_BOOT_TIME
}

scenario "no sentinel allows bootstrap to continue" test_no_sentinel_required
scenario "completed reboot clears sentinel automatically" test_removes_stale_sentinel
scenario "pending reboot blocks provisioning" test_detects_pending_reboot

if (( failures > 0 )); then
    printf '\n%d scenario(s) failed.\n' "$failures" >&2
    exit 1
fi

printf '\nAll scenarios passed.\n'
