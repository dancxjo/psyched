#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="${REPO_DIR:-$(cd "${SCRIPT_DIR}/../.." && pwd)}"
HOST_SHORT="${HOST:-$(hostname -s)}"

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

ARGS=()
if [[ -n "${FACES_CAMERA_TOPIC:-}" ]]; then
  ARGS+=("camera_topic:=${FACES_CAMERA_TOPIC}")
fi
if [[ -n "${FACES_FACES_TOPIC:-}" ]]; then
  ARGS+=("faces_topic:=${FACES_FACES_TOPIC}")
fi
if [[ -n "${FACES_FACE_DETECTED_TOPIC:-}" ]]; then
  ARGS+=("face_detected_topic:=${FACES_FACE_DETECTED_TOPIC}")
fi
if [[ -n "${FACES_TRIGGER_COOLDOWN_SEC:-}" ]]; then
  ARGS+=("trigger_cooldown_sec:=${FACES_TRIGGER_COOLDOWN_SEC}")
fi

exec ros2 launch faces face_detector.launch.py "${ARGS[@]}"
