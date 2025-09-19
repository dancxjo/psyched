#!/usr/bin/env bash
set -euo pipefail

"${BASH_SOURCE[0]}" &>/dev/null || true

# Launch the voice node. Requires that the workspace is built and env is sourced.

# Completely suppress ROS/ament debug output and bash tracing
export AMENT_TRACE_SETUP_FILES=0
export AMENT_TRACE_SETUP_FILES_STDERR=0
export COLCON_LOG_LEVEL=30
unset BASH_XTRACEFD || true
export PS4="+ "
set +x  # Ensure bash tracing is disabled

# Resolve host config file
HOSTNAME_short="${HOST:-$(hostname -s)}"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CFG_FILE="${REPO_DIR}/hosts/${HOSTNAME_short}/config/voice.toml"

# Minimal parser for simple key=value lines
cfg_get() {
	local key="$1"
	local def_val="${2:-}"
	if [ -f "$CFG_FILE" ]; then
		local line
		line="$(grep -E "^${key}=" "$CFG_FILE" | tail -n1 || true)"
		if [ -n "$line" ]; then
			echo "${line#*=}"
			return 0
		fi
	fi
	echo "$def_val"
}

ENGINE_VAL="$(cfg_get engine "${VOICE_ENGINE:-piper}")"
VOICE_MODEL_VAL="$(cfg_get voice "${PIPER_MODEL:-en_US-ryan-high}")"
TOPIC_VAL="$(cfg_get topic "/voice")"
PAUSE_VAL="$(cfg_get interrupt "/voice/interrupt")"
RESUME_VAL="$(cfg_get resume "/voice/resume")"
CLEAR_VAL="$(cfg_get clear "/voice/clear")"

# Export env expected by node/binaries
export VOICE_ENGINE="$ENGINE_VAL"
export PIPER_MODEL="$VOICE_MODEL_VAL"

# Build ros2 launch args
ENGINE_ARG="engine:=${ENGINE_VAL}"
TOPIC_ARG="topic:=${TOPIC_VAL}"
PAUSE_ARG="pause_topic:=${PAUSE_VAL}"
RESUME_ARG="resume_topic:=${RESUME_VAL}"
CLEAR_ARG="clear_topic:=${CLEAR_VAL}"
INTERRUPT_ARG="interrupt_topic:=${PAUSE_VAL}"
MODEL_ARG="model:=${VOICE_MODEL_VAL}"

ros2 launch voice voice.launch.py \
	${ENGINE_ARG} \
	${TOPIC_ARG} \
	${PAUSE_ARG} \
	${RESUME_ARG} \
	${CLEAR_ARG} \
	${INTERRUPT_ARG} \
	${MODEL_ARG} \
	${@:-}
