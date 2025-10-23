#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="${REPO_DIR:-$(cd "${SCRIPT_DIR}/../.." && pwd)}"
export REPO_DIR

if [[ -f "${REPO_DIR}/work/install/setup.bash" ]]; then
  set +u
  # shellcheck disable=SC1091
  source "${REPO_DIR}/work/install/setup.bash"
  set -u
fi

if [[ -f "${REPO_DIR}/work/install/conversant/share/conversant/local_setup.bash" ]]; then
  set +u
  # shellcheck disable=SC1091
  source "${REPO_DIR}/work/install/conversant/share/conversant/local_setup.bash"
  set -u
fi

MODE=${CONVERSANT_MODE:-balanced}
SILENCE_MS=${CONVERSANT_SILENCE_MS:-900}
THREAD_TTL=${CONVERSANT_THREAD_TTL_SECONDS:-180}
FILLERS=${CONVERSANT_FILLER_PHRASES:-}
LOCAL_LLM_URL=${CONVERSANT_LOCAL_LLM_URL:-}
MEMORY_TOPIC=${CONVERSANT_MEMORY_TOPIC:-/conversant/memory_event}
VAD_TOPIC=${CONVERSANT_VAD_TOPIC:-/ear/speech_active}
SILENCE_TOPIC=${CONVERSANT_SILENCE_TOPIC:-/ear/silence}
SPEECH_TOPIC=${CONVERSANT_SPEECH_TOPIC:-/voice}
PAUSE_TOPIC=${CONVERSANT_SPEECH_PAUSE_TOPIC:-/voice/pause}
RESUME_TOPIC=${CONVERSANT_SPEECH_RESUME_TOPIC:-/voice/resume}
CLEAR_TOPIC=${CONVERSANT_SPEECH_CLEAR_TOPIC:-/voice/clear}
HESITATE_TOPIC=${CONVERSANT_HESITATE_TOPIC:-/conversant/hesitate}
CONCERN_TOPIC=${CONVERSANT_CONCERN_TOPIC:-/conversant/concern}
CONTROL_TOPIC=${CONVERSANT_TURN_CONTROL_TOPIC:-/conversant/turn_control}
SPOKEN_TOPIC=${CONVERSANT_SPOKEN_TOPIC:-/voice/spoken}

LAUNCH_ARGS=(
  "mode:=${MODE}"
  "silence_ms:=${SILENCE_MS}"
  "thread_ttl_seconds:=${THREAD_TTL}"
  "memory_topic:=${MEMORY_TOPIC}"
  "vad_topic:=${VAD_TOPIC}"
  "silence_topic:=${SILENCE_TOPIC}"
  "speech_topic:=${SPEECH_TOPIC}"
  "pause_topic:=${PAUSE_TOPIC}"
  "resume_topic:=${RESUME_TOPIC}"
  "clear_topic:=${CLEAR_TOPIC}"
  "hesitate_topic:=${HESITATE_TOPIC}"
  "concern_topic:=${CONCERN_TOPIC}"
  "turn_control_topic:=${CONTROL_TOPIC}"
  "spoken_topic:=${SPOKEN_TOPIC}"
)

if [[ -n "${FILLERS}" ]]; then
  LAUNCH_ARGS+=("filler_phrases:=${FILLERS}")
fi

if [[ -n "${LOCAL_LLM_URL}" ]]; then
  LAUNCH_ARGS+=("local_llm_url:=${LOCAL_LLM_URL}")
fi

exec ros2 launch conversant conversant.launch.py "${LAUNCH_ARGS[@]}"
