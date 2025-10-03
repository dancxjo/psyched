#!/usr/bin/env bash
set -euo pipefail

BACKEND=${VOICE_BACKEND:-print}
VOICE_INPUT_TOPIC=${VOICE_INPUT_TOPIC:-/voice/text}
VOICE_SPOKEN_TOPIC=${VOICE_SPOKEN_TOPIC:-/voice/spoken}
VOICE_PAUSE_TOPIC=${VOICE_PAUSE_TOPIC:-/voice/pause}
VOICE_RESUME_TOPIC=${VOICE_RESUME_TOPIC:-/voice/resume}
VOICE_CLEAR_TOPIC=${VOICE_CLEAR_TOPIC:-/voice/clear}

ros2 run voice speech_service \
  --ros-args \
    -p backend:="${BACKEND}" \
    -p input_topic:="${VOICE_INPUT_TOPIC}" \
    -p spoken_topic:="${VOICE_SPOKEN_TOPIC}" \
    -p pause_topic:="${VOICE_PAUSE_TOPIC}" \
    -p resume_topic:="${VOICE_RESUME_TOPIC}" \
    -p clear_topic:="${VOICE_CLEAR_TOPIC}" &

wait -n
