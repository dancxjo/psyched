#!/usr/bin/env bash
set -euo pipefail

# Environment is set up by psh before calling this script
# PSYCHED_WORKSPACE_DIR and ROS environment should already be available

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
WORKSPACE_DIR="${PSYCHED_WORKSPACE_DIR:-${ROOT_DIR}/work}"
COCKPIT_PORT="${PILOT_COCKPIT_PORT:-8088}"

export PILOT_COCKPIT_PORT="${COCKPIT_PORT}"

add_pythonpath_dir() {
  local dir="$1"
  if [[ -z "${dir}" || ! -d "${dir}" ]]; then
    return
  fi
  case ":${PYTHONPATH:-}:" in
    *:"${dir}":*) ;;
    *)
      if [[ -n "${PYTHONPATH:-}" ]]; then
        export PYTHONPATH="${PYTHONPATH}:${dir}"
      else
        export PYTHONPATH="${dir}"
      fi
      ;;
  esac
}

USER_SITE_DIR="$(python3 -c 'import site; print(site.getusersitepackages())' 2>/dev/null || true)"
add_pythonpath_dir "${USER_SITE_DIR}"
add_pythonpath_dir "/usr/lib/python3/dist-packages"

# shellcheck disable=SC2317
cleanup() {
  if [[ -n "${COCKPIT_PID:-}" ]] && kill -0 "${COCKPIT_PID}" >/dev/null 2>&1; then
    echo "Stopping Pilot cockpit (PID ${COCKPIT_PID})..."
    kill "${COCKPIT_PID}" 2>/dev/null || true
    wait "${COCKPIT_PID}" 2>/dev/null || true
    COCKPIT_PID=""
  fi
}

trap cleanup EXIT INT TERM

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 CLI not found. Source your ROS workspace before launching the pilot module." >&2
  exit 1
fi

PILOT_PACKAGE_XML="${WORKSPACE_DIR}/install/pilot/share/pilot/package.xml"
if [[ ! -f "${PILOT_PACKAGE_XML}" ]]; then
  echo "pilot ROS package is missing (expected ${PILOT_PACKAGE_XML}). Run 'colcon build --packages-select pilot' before launching." >&2
  exit 1
fi

echo "Starting Pilot cockpit unified server on port ${COCKPIT_PORT}..."
(
  cd "${ROOT_DIR}" &&
    ros2 run pilot cockpit \
      --port "${COCKPIT_PORT}" \
      --www-dir "${ROOT_DIR}/modules/pilot/www" \
      --hosts-dir "${ROOT_DIR}/hosts" \
      --log-level "${PILOT_LOG_LEVEL:-info}"
) &
COCKPIT_PID=$!

sleep 2
if ! kill -0 "${COCKPIT_PID}" >/dev/null 2>&1; then
  echo "Pilot cockpit failed to start." >&2
  wait "${COCKPIT_PID}" || true
  exit 1
fi

echo "Pilot cockpit is running (PID ${COCKPIT_PID})."
echo "Websocket: ws://0.0.0.0:${COCKPIT_PORT}/ws"
echo "HTTP UI: http://0.0.0.0:${HTTP_PORT}"
echo "Press Ctrl+C to stop."

wait "${COCKPIT_PID}"
