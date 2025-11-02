#!/usr/bin/env bash
set -euo pipefail

# Launch the Eye module Kinect stack.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="${REPO_DIR:-$(cd "${SCRIPT_DIR}/../.." && pwd)}"
HOST_SHORT="${HOST:-$(hostname -s)}"

CONFIG_FILE="${EYE_CONFIG_FILE:-${REPO_DIR}/hosts/${HOST_SHORT}.toml}"
PARAM_FILE="${REPO_DIR}/hosts/${HOST_SHORT}/config/eye.yaml"

RGB_DEFAULT="/camera/color/image_raw"
DEPTH_DEFAULT="/camera/depth/image_raw"
derive_camera_info() {
  local topic="$1"
  if [[ "${topic}" == */* ]]; then
    echo "${topic%/*}/camera_info"
  else
    echo "camera_info"
  fi
}

normalize_bool() {
  local value="${1:-}"
  local default="${2:-false}"
  if [[ -z "${value}" ]]; then
    echo "${default}"
    return
  fi
  case "${value,,}" in
    1|true|yes|on) echo "true" ;;
    0|false|no|off) echo "false" ;;
    *) echo "${default}" ;;
  esac
}

if [[ -f "${CONFIG_FILE}" ]]; then
  eval "$(python3 - "${CONFIG_FILE}" <<'PY'
import shlex
import sys
from pathlib import Path

try:
    try:
        import tomllib  # type: ignore
    except ModuleNotFoundError:  # pragma: no cover - best effort fallback
        import tomli as tomllib  # type: ignore
except ModuleNotFoundError:
    raise SystemExit(0)

def emit(name: str, value):
    if value is None or value == "":
        return
    if isinstance(value, bool):
        text = "true" if value else "false"
    else:
        text = str(value)
    print(f'{name}={shlex.quote(text)}')

path = Path(sys.argv[1])
try:
    data = tomllib.loads(path.read_text(encoding="utf-8"))
except Exception:
    raise SystemExit(0)

def get(*path, default=None):
    cursor = data
    for key in path:
        if not isinstance(cursor, dict):
            return default
        cursor = cursor.get(key)
    return cursor if cursor is not None else default

emit("CFG_RGB_TOPIC", get("config", "mod", "eye", "launch", "arguments", "rgb_topic"))
emit("CFG_DEPTH_TOPIC", get("config", "mod", "eye", "launch", "arguments", "depth_topic"))
emit("CFG_KINECT_ENABLED", get("config", "mod", "eye", "launch", "kinect", "enabled"))
emit("CFG_KINECT_PARAMS_FILE", get("config", "mod", "eye", "launch", "kinect", "params_file"))
emit("CFG_RGB_INFO_TOPIC", get("config", "mod", "eye", "launch", "kinect", "rgb_info_topic"))
emit("CFG_DEPTH_INFO_TOPIC", get("config", "mod", "eye", "launch", "kinect", "depth_info_topic"))
PY
  )"
fi

RGB_TOPIC="${EYE_RGB_TOPIC:-${CFG_RGB_TOPIC:-${RGB_DEFAULT}}}"
DEPTH_TOPIC="${EYE_DEPTH_TOPIC:-${CFG_DEPTH_TOPIC:-${DEPTH_DEFAULT}}}"
RGB_INFO_TOPIC="${EYE_RGB_INFO_TOPIC:-${CFG_RGB_INFO_TOPIC:-$(derive_camera_info "${RGB_TOPIC}")}}"
DEPTH_INFO_TOPIC="${EYE_DEPTH_INFO_TOPIC:-${CFG_DEPTH_INFO_TOPIC:-$(derive_camera_info "${DEPTH_TOPIC}")}}"

KINECT_PARAMS="${EYE_KINECT_PARAMS_FILE:-${CFG_KINECT_PARAMS_FILE:-}}"
if [[ -z "${KINECT_PARAMS}" && -f "${PARAM_FILE}" ]]; then
  KINECT_PARAMS="${PARAM_FILE}"
fi

USE_KINECT="$(normalize_bool "${EYE_USE_KINECT:-${CFG_KINECT_ENABLED:-true}}" "true")"

CMD=("ros2" "launch" "psyched_eye" "eye.launch.py")
ARGS=(
  "use_kinect:=${USE_KINECT}"
  "kinect_rgb_topic:=${RGB_TOPIC}"
  "kinect_rgb_info_topic:=${RGB_INFO_TOPIC}"
  "kinect_depth_topic:=${DEPTH_TOPIC}"
  "kinect_depth_info_topic:=${DEPTH_INFO_TOPIC}"
)

if [[ -n "${KINECT_PARAMS}" ]]; then
  ARGS+=("kinect_params_file:=${KINECT_PARAMS}")
fi

exec "${CMD[@]}" "${ARGS[@]}"
