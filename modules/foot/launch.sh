#!/usr/bin/env bash
set -euo pipefail

HOSTNAME_short="${HOST:-$(hostname -s)}"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CFG_FILE="${REPO_DIR}/hosts/${HOSTNAME_short}/config/foot.toml"

cfg_get() {
	local key="$1"; local def_val="${2:-}"
	if [ -f "$CFG_FILE" ]; then
		local line
		line="$(grep -E "^${key}=" "$CFG_FILE" | tail -n1 || true)"
		if [ -n "$line" ]; then echo "${line#*=}"; return 0; fi
	fi
	echo "$def_val"
}

TOPIC_VAL="$(cfg_get topic "/foot")"

# Export for any nodes that may read env
export FOOT_TOPIC="$TOPIC_VAL"

# If the create_bringup supports remappings/params, we could pass them here.
ros2 launch create_bringup create_1.launch
