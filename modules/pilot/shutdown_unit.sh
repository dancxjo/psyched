#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
WORKSPACE_ENV="${ROOT_DIR}/env/psyched_env.sh"

if [[ -f "${WORKSPACE_ENV}" ]]; then
	# shellcheck disable=SC1090
	source "${WORKSPACE_ENV}"
fi

WORKSPACE_DIR="${PSYCHED_WORKSPACE_DIR:-${ROOT_DIR}/work}"

TIMEOUT=${TIMEOUT:-10}

terminate_process_group() {
	local description=$1
	shift
	local patterns=("$@")

	local matched=0

	for pattern in "${patterns[@]}"; do
		if pgrep -f "${pattern}" >/dev/null 2>&1; then
			if (( matched == 0 )); then
				echo "[pilot/shutdown] Sending SIGTERM to ${description}"
			fi
			matched=1
			pkill -TERM -f "${pattern}" || true
		fi
	done

	if (( matched == 0 )); then
		echo "[pilot/shutdown] ${description} not running"
		return 0
	fi

	for ((i = 0; i < TIMEOUT; i++)); do
		sleep 1
		local still_running=0
		for pattern in "${patterns[@]}"; do
			if pgrep -f "${pattern}" >/dev/null 2>&1; then
				still_running=1
				break
			fi
		done

		if (( still_running == 0 )); then
			echo "[pilot/shutdown] ${description} stopped"
			return 0
		fi
	done

	echo "[pilot/shutdown] Forcing SIGKILL for ${description}"
	for pattern in "${patterns[@]}"; do
		pkill -KILL -f "${pattern}" || true
	done
}

terminate_process_group \
        "Pilot cockpit" \
        "ros2 run pilot cockpit" \
        "python3 -m pilot.cockpit.cli"

echo "[pilot/shutdown] Shutdown cleanup complete"
