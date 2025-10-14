#!/usr/bin/env bash
set -euo pipefail

BACKEND=${VOICE_BACKEND:-coqui}
VOICE_INPUT_TOPIC=${VOICE_INPUT_TOPIC:-/voice}
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

LAUNCH_ARGS=(
  "backend:=${BACKEND}"
  "input_topic:=${VOICE_INPUT_TOPIC}"
  "spoken_topic:=${VOICE_SPOKEN_TOPIC}"
  "pause_topic:=${VOICE_PAUSE_TOPIC}"
  "resume_topic:=${VOICE_RESUME_TOPIC}"
  "clear_topic:=${VOICE_CLEAR_TOPIC}"
)

if [[ -n "${VOICE_TTS_URL}" ]]; then
  LAUNCH_ARGS+=("tts_url:=${VOICE_TTS_URL}")
else
  if [[ -n "${VOICE_TTS_SCHEME}" ]]; then
    LAUNCH_ARGS+=("tts_scheme:=${VOICE_TTS_SCHEME}")
  fi
  if [[ -n "${VOICE_TTS_HOST}" ]]; then
    LAUNCH_ARGS+=("tts_host:=${VOICE_TTS_HOST}")
  fi
  if [[ -n "${VOICE_TTS_PORT}" ]]; then
    LAUNCH_ARGS+=("tts_port:=${VOICE_TTS_PORT}")
  fi
  if [[ -n "${VOICE_TTS_PATH}" ]]; then
    LAUNCH_ARGS+=("tts_path:=${VOICE_TTS_PATH}")
  fi
fi

if [[ -n "${VOICE_TTS_SPEAKER}" ]]; then
  LAUNCH_ARGS+=("tts_speaker:=${VOICE_TTS_SPEAKER}")
fi

if [[ -n "${VOICE_TTS_LANGUAGE}" ]]; then
  LAUNCH_ARGS+=("tts_language:=${VOICE_TTS_LANGUAGE}")
fi

ros2 launch voice speech.launch.py \
  "${LAUNCH_ARGS[@]}" &

wait -n
