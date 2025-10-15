#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="${REPO_DIR:-$(cd "${SCRIPT_DIR}/../.." && pwd)}"
export REPO_DIR

if [[ -f "${REPO_DIR}/work/install/setup.bash" ]]; then
  # colcon setup scripts expect some env vars to be unset.
  set +u
  # shellcheck disable=SC1091
  source "${REPO_DIR}/work/install/setup.bash"
  set -u
fi

if [[ -f "${REPO_DIR}/work/install/ear/share/ear/local_setup.bash" ]]; then
  set +u
  # shellcheck disable=SC1091
  source "${REPO_DIR}/work/install/ear/share/ear/local_setup.bash"
  set -u
fi

BACKEND=${EAR_BACKEND:-console}
HOLE_TOPIC=${EAR_HOLE_TOPIC:-/ear/hole}
AUDIO_TOPIC=${EAR_AUDIO_TOPIC:-/audio/raw}
TEXT_TOPIC=${EAR_TEXT_TOPIC:-}
SERVICE_URI=${EAR_SERVICE_URI:-}
AUDIO_SAMPLE_RATE=${EAR_AUDIO_SAMPLE_RATE:-16000}
AUDIO_CHANNELS=${EAR_AUDIO_CHANNELS:-1}
WHISPER_MODEL=${EAR_FASTER_WHISPER_MODEL:-}
WHISPER_DEVICE=${EAR_FASTER_WHISPER_DEVICE:-}
WHISPER_COMPUTE=${EAR_FASTER_WHISPER_COMPUTE_TYPE:-}
WHISPER_LANGUAGE=${EAR_FASTER_WHISPER_LANGUAGE:-}
WHISPER_BEAM_SIZE=${EAR_FASTER_WHISPER_BEAM_SIZE:-5}
ENABLE_AUDIO_CAPTURE=${EAR_AUDIO_CAPTURE_ENABLED:-true}
AUDIO_SOURCE_COMMAND=${EAR_AUDIO_SOURCE_COMMAND:-}
AUDIO_CHUNK_DURATION=${EAR_AUDIO_CHUNK_DURATION_MS:-}
AUDIO_CHUNK_SIZE=${EAR_AUDIO_CHUNK_SIZE:-}
AUDIO_RESTART_DELAY=${EAR_AUDIO_RESTART_DELAY:-}
AUDIO_RELIABILITY=${EAR_AUDIO_RELIABILITY:-reliable}
SPEECH_TOPIC=${EAR_SPEECH_TOPIC:-/ear/speech_active}
VAD_FRAME_DURATION=${EAR_VAD_FRAME_DURATION_MS:-}
VAD_AGGRESSIVENESS=${EAR_VAD_AGGRESSIVENESS:-}
VAD_SMOOTHING=${EAR_VAD_SMOOTHING_WINDOW:-}
VAD_PUBLISH_ON_CHANGE=${EAR_VAD_PUBLISH_ON_CHANGE:-true}
SILENCE_TOPIC=${EAR_SILENCE_TOPIC:-/ear/silence}
SILENCE_SAMPLE_WIDTH=${EAR_SAMPLE_WIDTH:-2}
SILENCE_THRESHOLD=${EAR_SILENCE_RMS_THRESHOLD:-}
SILENCE_AVERAGE_WINDOW=${EAR_SILENCE_AVERAGE_WINDOW:-}
SILENCE_PUBLISH_ON_CHANGE=${EAR_SILENCE_PUBLISH_ON_CHANGE:-true}

LAUNCH_ARGS=(
  backend:="${BACKEND}"
  enable_audio_capture:="${ENABLE_AUDIO_CAPTURE}"
  audio_reliability:="${AUDIO_RELIABILITY}"
  transcript_topic:="${HOLE_TOPIC}"
  audio_topic:="${AUDIO_TOPIC}"
  sample_rate:="${AUDIO_SAMPLE_RATE}"
  channels:="${AUDIO_CHANNELS}"
  speech_topic:="${SPEECH_TOPIC}"
  silence_topic:="${SILENCE_TOPIC}"
  sample_width:="${SILENCE_SAMPLE_WIDTH}"
)

if [[ -n "${TEXT_TOPIC}" ]]; then
  LAUNCH_ARGS+=(text_input_topic:="${TEXT_TOPIC}")
fi
if [[ -n "${SERVICE_URI}" ]]; then
  LAUNCH_ARGS+=(service_uri:="${SERVICE_URI}")
fi
if [[ -n "${AUDIO_SOURCE_COMMAND}" ]]; then
  LAUNCH_ARGS+=(audio_source_command:="${AUDIO_SOURCE_COMMAND}")
fi
if [[ -n "${AUDIO_CHUNK_DURATION}" ]]; then
  LAUNCH_ARGS+=(chunk_duration_ms:="${AUDIO_CHUNK_DURATION}")
fi
if [[ -n "${AUDIO_CHUNK_SIZE}" ]]; then
  LAUNCH_ARGS+=(chunk_size:="${AUDIO_CHUNK_SIZE}")
fi
if [[ -n "${AUDIO_RESTART_DELAY}" ]]; then
  LAUNCH_ARGS+=(restart_delay:="${AUDIO_RESTART_DELAY}")
fi
if [[ -n "${WHISPER_MODEL}" ]]; then
  LAUNCH_ARGS+=(faster_whisper_model:="${WHISPER_MODEL}")
fi
if [[ -n "${WHISPER_DEVICE}" ]]; then
  LAUNCH_ARGS+=(faster_whisper_device:="${WHISPER_DEVICE}")
fi
if [[ -n "${WHISPER_COMPUTE}" ]]; then
  LAUNCH_ARGS+=(faster_whisper_compute_type:="${WHISPER_COMPUTE}")
fi
if [[ -n "${WHISPER_LANGUAGE}" ]]; then
  LAUNCH_ARGS+=(faster_whisper_language:="${WHISPER_LANGUAGE}")
fi
if [[ -n "${WHISPER_BEAM_SIZE}" ]]; then
  LAUNCH_ARGS+=(faster_whisper_beam_size:="${WHISPER_BEAM_SIZE}")
fi
if [[ -n "${VAD_FRAME_DURATION}" ]]; then
  LAUNCH_ARGS+=(vad_frame_duration_ms:="${VAD_FRAME_DURATION}")
fi
if [[ -n "${VAD_AGGRESSIVENESS}" ]]; then
  LAUNCH_ARGS+=(vad_aggressiveness:="${VAD_AGGRESSIVENESS}")
fi
if [[ -n "${VAD_SMOOTHING}" ]]; then
  LAUNCH_ARGS+=(vad_smoothing_window:="${VAD_SMOOTHING}")
fi
if [[ -n "${VAD_PUBLISH_ON_CHANGE}" ]]; then
  LAUNCH_ARGS+=(vad_publish_on_change:="${VAD_PUBLISH_ON_CHANGE}")
fi
if [[ -n "${SILENCE_THRESHOLD}" ]]; then
  LAUNCH_ARGS+=(silence_rms_threshold:="${SILENCE_THRESHOLD}")
fi
if [[ -n "${SILENCE_AVERAGE_WINDOW}" ]]; then
  LAUNCH_ARGS+=(silence_average_window:="${SILENCE_AVERAGE_WINDOW}")
fi
if [[ -n "${SILENCE_PUBLISH_ON_CHANGE}" ]]; then
  LAUNCH_ARGS+=(silence_publish_on_change:="${SILENCE_PUBLISH_ON_CHANGE}")
fi

ros2 launch ear ear.launch.py "${LAUNCH_ARGS[@]}" &

wait -n
