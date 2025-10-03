#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
FRONTEND_DIR="${ROOT_DIR}/modules/pilot/frontend"
FRONTEND_DEV_TS="${FRONTEND_DIR}/dev.ts"
WORKSPACE_ENV="${ROOT_DIR}/workspace_env.sh"

if [[ -f "${WORKSPACE_ENV}" ]]; then
	# shellcheck disable=SC1090
	source "${WORKSPACE_ENV}"
fi

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
        "rosbridge websocket" \
        "rosbridge_websocket_launch.xml" \
        "ros2 launch rosbridge_server" \
        "rosbridge_server"

terminate_process_group \
        "web_video_server" \
        "ros2 run web_video_server web_video_server" \
        "web_video_server"

terminate_process_group \
        "Pilot Fresh dev server" \
	"deno task dev" \
	"deno run -A --watch=static/,routes/ dev.ts" \
	"${FRONTEND_DEV_TS}"

echo "[pilot/shutdown] Shutdown cleanup complete"
