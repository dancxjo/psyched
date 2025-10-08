#!/usr/bin/env bash
set -euo pipefail

BACKEND=${VOICE_BACKEND:-print}
VOICE_INPUT_TOPIC=${VOICE_INPUT_TOPIC:-/voice/text}
VOICE_SPOKEN_TOPIC=${VOICE_SPOKEN_TOPIC:-/voice/spoken}
VOICE_PAUSE_TOPIC=${VOICE_PAUSE_TOPIC:-/voice/pause}
VOICE_RESUME_TOPIC=${VOICE_RESUME_TOPIC:-/voice/resume}
VOICE_CLEAR_TOPIC=${VOICE_CLEAR_TOPIC:-/voice/clear}
VOICE_TTS_URL=${VOICE_TTS_URL:-}
VOICE_TTS_SCHEME=${VOICE_TTS_SCHEME:-}
VOICE_TTS_HOST=${VOICE_TTS_HOST:-}
VOICE_TTS_PORT=${VOICE_TTS_PORT:-}
VOICE_TTS_PATH=${VOICE_TTS_PATH:-}
VOICE_TTS_SPEAKER=${VOICE_TTS_SPEAKER:-}
VOICE_TTS_LANGUAGE=${VOICE_TTS_LANGUAGE:-}

ROS_ARGS=(
  -p backend:="${BACKEND}"
  -p input_topic:="${VOICE_INPUT_TOPIC}"
  -p spoken_topic:="${VOICE_SPOKEN_TOPIC}"
  -p pause_topic:="${VOICE_PAUSE_TOPIC}"
  -p resume_topic:="${VOICE_RESUME_TOPIC}"
  -p clear_topic:="${VOICE_CLEAR_TOPIC}"
)

if [[ -n "${VOICE_TTS_URL}" ]]; then
  ROS_ARGS+=(-p tts_url:="${VOICE_TTS_URL}")
else
  if [[ -n "${VOICE_TTS_SCHEME}" ]]; then
    ROS_ARGS+=(-p tts_scheme:="${VOICE_TTS_SCHEME}")
  fi
  if [[ -n "${VOICE_TTS_HOST}" ]]; then
    ROS_ARGS+=(-p tts_host:="${VOICE_TTS_HOST}")
  fi
  if [[ -n "${VOICE_TTS_PORT}" ]]; then
    ROS_ARGS+=(-p tts_port:=${VOICE_TTS_PORT})
  fi
  if [[ -n "${VOICE_TTS_PATH}" ]]; then
    ROS_ARGS+=(-p tts_path:="${VOICE_TTS_PATH}")
  fi
fi

if [[ -n "${VOICE_TTS_SPEAKER}" ]]; then
  ROS_ARGS+=(-p tts_speaker:="${VOICE_TTS_SPEAKER}")
fi

if [[ -n "${VOICE_TTS_LANGUAGE}" ]]; then
  ROS_ARGS+=(-p tts_language:="${VOICE_TTS_LANGUAGE}")
fi

ros2 run voice speech_service \
  --ros-args \
    "${ROS_ARGS[@]}" &

wait -n
