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
AUDIO_SAMPLE_RATE=${EAR_AUDIO_SAMPLE_RATE:-}
AUDIO_CHANNELS=${EAR_AUDIO_CHANNELS:-}
WHISPER_MODEL=${EAR_FASTER_WHISPER_MODEL:-}
WHISPER_DEVICE=${EAR_FASTER_WHISPER_DEVICE:-}
WHISPER_COMPUTE=${EAR_FASTER_WHISPER_COMPUTE_TYPE:-}
WHISPER_LANGUAGE=${EAR_FASTER_WHISPER_LANGUAGE:-}
WHISPER_BEAM_SIZE=${EAR_FASTER_WHISPER_BEAM_SIZE:-}

ROS_ARGS=(
  -p backend:="${BACKEND}"
  -p hole_topic:="${HOLE_TOPIC}"
)

if [[ -n "${AUDIO_TOPIC}" ]]; then
  ROS_ARGS+=(-p audio_topic:="${AUDIO_TOPIC}")
fi
if [[ -n "${TEXT_TOPIC}" ]]; then
  ROS_ARGS+=(-p text_input_topic:="${TEXT_TOPIC}")
fi
if [[ -n "${SERVICE_URI}" ]]; then
  ROS_ARGS+=(-p service_uri:="${SERVICE_URI}")
fi
if [[ -n "${AUDIO_SAMPLE_RATE}" ]]; then
  ROS_ARGS+=(-p audio_sample_rate:=${AUDIO_SAMPLE_RATE})
fi
if [[ -n "${AUDIO_CHANNELS}" ]]; then
  ROS_ARGS+=(-p audio_channels:=${AUDIO_CHANNELS})
fi
if [[ -n "${WHISPER_MODEL}" ]]; then
  ROS_ARGS+=(-p faster_whisper_model:="${WHISPER_MODEL}")
fi
if [[ -n "${WHISPER_DEVICE}" ]]; then
  ROS_ARGS+=(-p faster_whisper_device:="${WHISPER_DEVICE}")
fi
if [[ -n "${WHISPER_COMPUTE}" ]]; then
  ROS_ARGS+=(-p faster_whisper_compute_type:="${WHISPER_COMPUTE}")
fi
if [[ -n "${WHISPER_LANGUAGE}" ]]; then
  ROS_ARGS+=(-p faster_whisper_language:="${WHISPER_LANGUAGE}")
fi
if [[ -n "${WHISPER_BEAM_SIZE}" ]]; then
  ROS_ARGS+=(-p faster_whisper_beam_size:=${WHISPER_BEAM_SIZE})
fi

ros2 run ear ear_service --ros-args "${ROS_ARGS[@]}" &

wait -n
