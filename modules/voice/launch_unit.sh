#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="${REPO_DIR:-$(cd "${SCRIPT_DIR}/../.." && pwd)}"
export REPO_DIR

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

if [[ -f "${REPO_DIR}/work/install/setup.bash" ]]; then
  # colcon's setup scripts read unset vars, so relax -u while sourcing.
  set +u
  # shellcheck disable=SC1091
  source "${REPO_DIR}/work/install/setup.bash"
  set -u
fi

if [[ -f "${REPO_DIR}/work/install/voice/share/voice/local_setup.bash" ]]; then
  set +u
  # shellcheck disable=SC1091
  source "${REPO_DIR}/work/install/voice/share/voice/local_setup.bash"
  set -u
fi

find_and_prepend_websockets() {
  # Candidate site-packages where a newer 'websockets' may be installed.
  candidates=(
    "${REPO_DIR}/work/install/voice/lib/python3.12/site-packages"
    "/opt/ros/*/colcon-venv/lib/python3.12/site-packages"
  )

  for cand in "${candidates[@]}"; do
    # Expand globs
    for path in $cand; do
      if [[ -d "$path" ]]; then
        # Check whether importing websockets from this path yields a __version__ >= 12
        if python3 - <<PY >/dev/null 2>&1
import sys
sys.path.insert(0, "$path")
try:
    import websockets
    ver = getattr(websockets, '__version__', '0')
    # simple numeric check: major version >=12
    major = int(ver.split('.')[0]) if ver and ver.split('.')[0].isdigit() else 0
    sys.exit(0) if major >= 12 else sys.exit(2)
except Exception:
    sys.exit(3)
PY
        then
          echo "[voice/launch] Prepending $path to PYTHONPATH to prefer modern websockets" >&2
          export PYTHONPATH="$path:${PYTHONPATH:-}"
          return 0
        fi
      fi
    done
  done
  return 1
}

# Try to prefer a workspace/venv-installed websockets package if present.
find_and_prepend_websockets || true

ros2 launch voice speech.launch.py \
  "${LAUNCH_ARGS[@]}" &

wait -n
