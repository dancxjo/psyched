#!/usr/bin/env bash
set -euo pipefail

# Launch the voice node. Requires that the workspace is built and env is sourced.

ENGINE_ARG=""
if [[ -n "${VOICE_ENGINE:-}" ]]; then
	ENGINE_ARG="engine:=${VOICE_ENGINE}"
fi

ros2 launch voice voice.launch.py ${ENGINE_ARG} ${@:-}
