#!/usr/bin/env bash
set -euo pipefail

# Launch the voice node. Requires that the workspace is built and env is sourced.

# Determine engine (default to espeak)
ENGINE_VAL="${VOICE_ENGINE:-espeak}"
ENGINE_ARG="engine:=${ENGINE_VAL}"

# Determine espeak voice (default to mb-en1)
ESPEAK_VOICE_VAL="${ESPEAK_VOICE:-mb-en1}"
ESPEAK_VOICE_ARG="espeak_voice:=${ESPEAK_VOICE_VAL}"

ros2 launch voice voice.launch.py ${ENGINE_ARG} ${ESPEAK_VOICE_ARG} ${@:-}
