#!/usr/bin/env bash
set -euo pipefail

HOSTNAME_short="${HOST:-$(hostname -s)}"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ENV_FILE="${REPO_DIR}/hosts/${HOSTNAME_short}/config/foot.env"

if [ -f "$ENV_FILE" ]; then
  # shellcheck disable=SC1090
  . "$ENV_FILE"
fi

TOPIC_VAL="${FOOT_TOPIC:-/foot}"

# Export for any nodes that may read env
export FOOT_TOPIC="$TOPIC_VAL"

# If the create_bringup supports remappings/params, we could pass them here.
ros2 launch create_bringup create_1.launch
