#!/usr/bin/env bash
set -euo pipefail

# Launch the Kinect ROS 2 driver with optional host-specific parameters.
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

if [[ -f "${CONFIG_FILE}" ]]; then
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
    "config.mod.eye.launch.arguments.rgb_topic",
    "config.mod.nav.launch.arguments.kinect_rgb_topic",
))
print(pick(
    "config.topics.camera.depth",
    "config.mod.eye.launch.arguments.depth_topic",
    "config.mod.nav.launch.arguments.kinect_depth_topic",
))
PY
  )
  if [[ ${#CONFIG_TOPICS[@]} -ge 1 && -n "${CONFIG_TOPICS[0]}" ]]; then
    RGB_DEFAULT="${CONFIG_TOPICS[0]}"
  fi
  if [[ ${#CONFIG_TOPICS[@]} -ge 2 && -n "${CONFIG_TOPICS[1]}" ]]; then
    DEPTH_DEFAULT="${CONFIG_TOPICS[1]}"
  fi
fi

RGB_TOPIC="${EYE_RGB_TOPIC:-${RGB_DEFAULT}}"
DEPTH_TOPIC="${EYE_DEPTH_TOPIC:-${DEPTH_DEFAULT}}"
RGB_INFO_TOPIC="${EYE_RGB_INFO_TOPIC:-$(derive_camera_info "${RGB_TOPIC}")}"
DEPTH_INFO_TOPIC="${EYE_DEPTH_INFO_TOPIC:-$(derive_camera_info "${DEPTH_TOPIC}")}"

CMD=("ros2" "run" "kinect_ros2" "kinect_ros2_node")
ROS_ARGS=()

if [[ -f "${PARAM_FILE}" ]]; then
  ROS_ARGS+=("--params-file" "${PARAM_FILE}")
fi

ROS_ARGS+=(
  "-r" "image_raw:=${RGB_TOPIC}"
  "-r" "camera_info:=${RGB_INFO_TOPIC}"
  "-r" "depth/image_raw:=${DEPTH_TOPIC}"
  "-r" "depth/camera_info:=${DEPTH_INFO_TOPIC}"
)

if [[ ${#ROS_ARGS[@]} -gt 0 ]]; then
  CMD+=("--ros-args" "${ROS_ARGS[@]}")
fi

exec "${CMD[@]}"
