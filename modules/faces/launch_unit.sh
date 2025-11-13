#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="${REPO_DIR:-$(cd "${SCRIPT_DIR}/../.." && pwd)}"
HOST_SHORT="${HOST:-$(hostname -s)}"
CONFIG_FILE="${FACES_CONFIG_FILE:-${REPO_DIR}/hosts/${HOST_SHORT}.toml}"

DEFAULT_CAMERA_TOPIC="/camera/color/image_raw"
DEFAULT_FACES_TOPIC="/vision/faces"
DEFAULT_FACE_DETECTED_TOPIC="/vision/face_detected"

CONFIG_CAMERA_TOPIC=""
CONFIG_FACES_TOPIC=""
CONFIG_FACE_DETECTED_TOPIC=""

if [[ -f "${CONFIG_FILE}" ]]; then
  # Prefer the shared camera topic overrides but fall back to the module defaults.
  mapfile -t CONFIG_TOPICS < <(python3 - "${CONFIG_FILE}" <<'PY'
import sys
from pathlib import Path

try:
  try:
    import tomllib  # type: ignore
  except ModuleNotFoundError:  # pragma: no cover - best effort fallback
    import tomli as tomllib  # type: ignore
except ModuleNotFoundError:
  print()
  print()
  print()
  raise SystemExit(0)

path = Path(sys.argv[1])
try:
  data = tomllib.loads(path.read_text(encoding="utf-8"))
except Exception:
  raise SystemExit(0)

def pick(*paths: str) -> str:
  for dotted in paths:
    cursor = data
    for key in dotted.split('.'):
      if not isinstance(cursor, dict):
        break
      cursor = cursor.get(key)
    else:
      if isinstance(cursor, str):
        return cursor.strip()
  return ""

print(pick(
  "config.topics.camera.rgb",
  "config.mod.faces.launch.arguments.camera_topic",
  "config.mod.eye.launch.arguments.rgb_topic",
  "config.mod.nav.launch.arguments.kinect_rgb_topic",
))
print(pick(
  "config.mod.faces.launch.arguments.faces_topic",
))
print(pick(
  "config.mod.faces.launch.arguments.face_detected_topic",
))
PY
  )
  if [[ ${#CONFIG_TOPICS[@]} -ge 1 && -n "${CONFIG_TOPICS[0]}" ]]; then
  CONFIG_CAMERA_TOPIC="${CONFIG_TOPICS[0]}"
  fi
  if [[ ${#CONFIG_TOPICS[@]} -ge 2 && -n "${CONFIG_TOPICS[1]}" ]]; then
  CONFIG_FACES_TOPIC="${CONFIG_TOPICS[1]}"
  fi
  if [[ ${#CONFIG_TOPICS[@]} -ge 3 && -n "${CONFIG_TOPICS[2]}" ]]; then
  CONFIG_FACE_DETECTED_TOPIC="${CONFIG_TOPICS[2]}"
  fi
fi

PARAM_FILE="${FACES_PARAMS_FILE:-}"
if [[ -z "${PARAM_FILE}" ]]; then
  LEGACY_YAML="${REPO_DIR}/hosts/${HOST_SHORT}/config/faces.yaml"
  if [[ -f "${LEGACY_YAML}" ]]; then
    PARAM_FILE="${LEGACY_YAML}"
  fi
fi

if [[ -n "${PARAM_FILE}" ]]; then
  exec ros2 launch faces face_detector.launch.py --ros-args --params-file "${PARAM_FILE}"
fi

CAMERA_TOPIC="${FACES_CAMERA_TOPIC:-${CONFIG_CAMERA_TOPIC:-${DEFAULT_CAMERA_TOPIC}}}"
FACES_TOPIC="${FACES_FACES_TOPIC:-${CONFIG_FACES_TOPIC:-${DEFAULT_FACES_TOPIC}}}"
FACE_DETECTED_TOPIC="${FACES_FACE_DETECTED_TOPIC:-${CONFIG_FACE_DETECTED_TOPIC:-${DEFAULT_FACE_DETECTED_TOPIC}}}"

ARGS=()
ARGS+=("camera_topic:=${CAMERA_TOPIC}")
ARGS+=("faces_topic:=${FACES_TOPIC}")
ARGS+=("face_detected_topic:=${FACE_DETECTED_TOPIC}")
if [[ -n "${FACES_TRIGGER_COOLDOWN_SEC:-}" ]]; then
  ARGS+=("trigger_cooldown_sec:=${FACES_TRIGGER_COOLDOWN_SEC}")
fi
if [[ -n "${FACES_SERVICE_URI:-}" ]]; then
  ARGS+=("embedding.service_uri:=${FACES_SERVICE_URI}")
fi
if [[ -n "${FACES_SERVICE_TIMEOUT_SEC:-}" ]]; then
  ARGS+=("embedding.service_timeout_sec:=${FACES_SERVICE_TIMEOUT_SEC}")
fi
if [[ -n "${FACES_SERVICE_MODEL:-}" ]]; then
  ARGS+=("embedding.service_model:=${FACES_SERVICE_MODEL}")
fi
if [[ -n "${FACES_SERVICE_NUM_JITTERS:-}" ]]; then
  ARGS+=("embedding.service_num_jitters:=${FACES_SERVICE_NUM_JITTERS}")
fi

exec ros2 launch faces face_detector.launch.py "${ARGS[@]}"
