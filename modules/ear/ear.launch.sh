#!/bin/bash
set -euo pipefail

coerce_int() {
  local value="$1"
  local default="$2"
  if [[ -z "${value}" ]]; then
    echo "${default}"
    return
  fi
  if [[ "${value}" =~ ^-?[0-9]+$ ]]; then
    echo "${value}"
    return
  fi
  echo "${default}"
}

DEVICE_ID_VAL=$(coerce_int "${EAR_DEVICE:-0}" 0)
RATE_VAL=$(coerce_int "${EAR_RATE:-44100}" 44100)
CHANNELS_VAL=$(coerce_int "${EAR_CHANNELS:-1}" 1)
CHUNK_VAL=$(coerce_int "${EAR_CHUNK:-1024}" 1024)
SILENCE_THRESHOLD_VAL="${EAR_SILENCE_THRESHOLD:-500.0}"

MODEL_VAL="${EAR_MODEL:-base}"
ASR_DEVICE_VAL="${EAR_ASR_DEVICE:-cpu}"
ASR_COMPUTE_VAL="${EAR_ASR_COMPUTE_TYPE:-int8}"
ASR_LANGUAGE_VAL="${EAR_LANGUAGE:-}"
ASR_BEAM_VAL=$(coerce_int "${EAR_BEAM_SIZE:-5}" 5)

REPO_DIR="${REPO_DIR:-$(cd "$(dirname "$0")/../.." && pwd)}"
export REPO_DIR

args=(
  ros2 launch ear ear.launch.py
  device_id:="${DEVICE_ID_VAL}"
  sample_rate:="${RATE_VAL}"
  channels:="${CHANNELS_VAL}"
  chunk_size:="${CHUNK_VAL}"
  silence_threshold:="${SILENCE_THRESHOLD_VAL}"
  model:="${MODEL_VAL}"
  device:="${ASR_DEVICE_VAL}"
  compute_type:="${ASR_COMPUTE_VAL}"
)

# Only add language argument when a value is present to avoid malformed
# launch args like 'language:=' which ros2 launch rejects.
if [[ -n "${ASR_LANGUAGE_VAL}" ]]; then
  args+=(language:="${ASR_LANGUAGE_VAL}")
fi

args+=(beam_size:="${ASR_BEAM_VAL}")

exec "${args[@]}"
