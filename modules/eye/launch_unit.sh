#!/usr/bin/env bash
set -euo pipefail

# Launch the Eye module camera stack (Kinect + optional USB/V4L cameras).
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="${REPO_DIR:-$(cd "${SCRIPT_DIR}/../.." && pwd)}"
HOST_SHORT="${HOST:-$(hostname -s)}"

CONFIG_FILE="${EYE_CONFIG_FILE:-${REPO_DIR}/hosts/${HOST_SHORT}.toml}"
PARAM_FILE="${REPO_DIR}/hosts/${HOST_SHORT}/config/eye.yaml"

RGB_DEFAULT="/camera/color/image_raw"
DEPTH_DEFAULT="/camera/depth/image_raw"
USB_IMAGE_DEFAULT="/eye/usb/image_raw"

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

emit("CFG_USB_ENABLED", get("config", "mod", "eye", "launch", "usb", "enabled"))
emit("CFG_USB_DEVICE", get("config", "mod", "eye", "launch", "usb", "device"))
emit("CFG_USB_FRAME_ID", get("config", "mod", "eye", "launch", "usb", "frame_id"))
emit("CFG_USB_WIDTH", get("config", "mod", "eye", "launch", "usb", "width"))
emit("CFG_USB_HEIGHT", get("config", "mod", "eye", "launch", "usb", "height"))
emit("CFG_USB_FPS", get("config", "mod", "eye", "launch", "usb", "fps"))
emit("CFG_USB_ENCODING", get("config", "mod", "eye", "launch", "usb", "encoding"))
emit("CFG_USB_IMAGE_TOPIC", get("config", "mod", "eye", "launch", "usb", "image_topic"))
emit("CFG_USB_INFO_TOPIC", get("config", "mod", "eye", "launch", "usb", "camera_info_topic"))

emit("CFG_FACES_SOURCE", get("config", "mod", "eye", "launch", "faces", "source"))
emit("CFG_FACES_FALLBACK", get("config", "mod", "eye", "launch", "faces", "fallback"))
emit("CFG_FACES_IMAGE_TOPIC", get("config", "mod", "eye", "launch", "faces", "image_topic"))
emit("CFG_FACES_INFO_TOPIC", get("config", "mod", "eye", "launch", "faces", "camera_info_topic"))
emit("CFG_FACES_ENABLE_ROUTER", get("config", "mod", "eye", "launch", "faces", "enable_router"))
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

USB_IMAGE_TOPIC="${EYE_USB_IMAGE_TOPIC:-${CFG_USB_IMAGE_TOPIC:-${USB_IMAGE_DEFAULT}}}"
USB_INFO_TOPIC_DEFAULT="$(derive_camera_info "${USB_IMAGE_TOPIC}")"
USB_INFO_TOPIC="${EYE_USB_INFO_TOPIC:-${CFG_USB_INFO_TOPIC:-${USB_INFO_TOPIC_DEFAULT}}}"

AUTO_USB_ENABLED="false"
AUTO_USB_DEVICE=""
if [[ -z "${EYE_USB_ENABLED:-}" && -z "${CFG_USB_ENABLED:-}" ]]; then
  if compgen -G "/dev/video*" >/dev/null 2>&1; then
    AUTO_USB_ENABLED="true"
    AUTO_USB_DEVICE="$(ls -1 /dev/video* 2>/dev/null | head -n1 || true)"
  fi
fi

USE_USB="$(normalize_bool "${EYE_USB_ENABLED:-${CFG_USB_ENABLED:-${AUTO_USB_ENABLED}}}" "${AUTO_USB_ENABLED}")"
USB_DEVICE_DEFAULT="${AUTO_USB_DEVICE:-/dev/video0}"
USB_DEVICE="${EYE_USB_DEVICE:-${CFG_USB_DEVICE:-${USB_DEVICE_DEFAULT}}}"
USB_FRAME_ID="${EYE_USB_FRAME_ID:-${CFG_USB_FRAME_ID:-usb_camera}}"
USB_WIDTH="${EYE_USB_WIDTH:-${CFG_USB_WIDTH:-640}}"
USB_HEIGHT="${EYE_USB_HEIGHT:-${CFG_USB_HEIGHT:-480}}"
USB_FPS="${EYE_USB_FPS:-${CFG_USB_FPS:-30.0}}"
USB_ENCODING="${EYE_USB_ENCODING:-${CFG_USB_ENCODING:-bgr8}}"

FACES_SOURCE="${EYE_FACES_SOURCE:-${CFG_FACES_SOURCE:-kinect}}"
FACES_FALLBACK="${EYE_FACES_FALLBACK:-${CFG_FACES_FALLBACK:-auto}}"
FACES_IMAGE_TOPIC="${EYE_FACES_IMAGE_TOPIC:-${CFG_FACES_IMAGE_TOPIC:-/faces/camera/image_raw}}"
FACES_INFO_TOPIC="${EYE_FACES_INFO_TOPIC:-${CFG_FACES_INFO_TOPIC:-/faces/camera/camera_info}}"
ENABLE_ROUTER="$(normalize_bool "${EYE_ENABLE_FACES_ROUTER:-${CFG_FACES_ENABLE_ROUTER:-true}}" "true")"

CMD=("ros2" "launch" "psyched_eye" "eye.launch.py")
ARGS=(
  "use_kinect:=${USE_KINECT}"
  "kinect_rgb_topic:=${RGB_TOPIC}"
  "kinect_rgb_info_topic:=${RGB_INFO_TOPIC}"
  "kinect_depth_topic:=${DEPTH_TOPIC}"
  "kinect_depth_info_topic:=${DEPTH_INFO_TOPIC}"
  "use_usb_camera:=${USE_USB}"
  "usb_device:=${USB_DEVICE}"
  "usb_frame_id:=${USB_FRAME_ID}"
  "usb_width:=${USB_WIDTH}"
  "usb_height:=${USB_HEIGHT}"
  "usb_fps:=${USB_FPS}"
  "usb_encoding:=${USB_ENCODING}"
  "usb_image_topic:=${USB_IMAGE_TOPIC}"
  "usb_camera_info_topic:=${USB_INFO_TOPIC}"
  "enable_faces_router:=${ENABLE_ROUTER}"
  "faces_source:=${FACES_SOURCE}"
  "faces_fallback_source:=${FACES_FALLBACK}"
  "faces_output_image_topic:=${FACES_IMAGE_TOPIC}"
  "faces_output_camera_info_topic:=${FACES_INFO_TOPIC}"
)

if [[ -n "${KINECT_PARAMS}" ]]; then
  ARGS+=("kinect_params_file:=${KINECT_PARAMS}")
fi

if [[ "${AUTO_USB_ENABLED}" == "true" && "${USE_USB}" == "true" ]]; then
  printf 'Auto-detected USB camera at %s; enabling USB stream.\n' "${USB_DEVICE}"
fi

exec "${CMD[@]}" "${ARGS[@]}"
