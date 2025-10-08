#!/usr/bin/env bash
set -euo pipefail

# Environment is set up by psh before calling this script
# PSYCHED_WORKSPACE_DIR and ROS environment should already be available

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
printf '%s start launch_unit\\n' "$(date -Iseconds)" >> /tmp/pilot_launch_debug.log
FRONTEND_DIR="${ROOT_DIR}/modules/pilot/frontend"
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
    echo "Stopping Pilot cockpit backend (PID ${COCKPIT_PID})..."
    kill "${COCKPIT_PID}" 2>/dev/null || true
    wait "${COCKPIT_PID}" 2>/dev/null || true
    COCKPIT_PID=""
  fi

  if [[ -n "${DENO_PID:-}" ]] && kill -0 "${DENO_PID}" >/dev/null 2>&1; then
    echo "Stopping Pilot frontend (PID ${DENO_PID})..."
    kill "${DENO_PID}" 2>/dev/null || true
    wait "${DENO_PID}" 2>/dev/null || true
    DENO_PID=""
  fi
}

trap cleanup EXIT INT TERM

monitor_children() {
  local status=0

  echo "Pilot cockpit backend PID ${COCKPIT_PID}"
  echo "Pilot frontend PID ${DENO_PID}"
  echo "Pilot cockpit backend and frontend are running. Press Ctrl+C to stop."

  while true; do
    local pids=()
    if [[ -n "${COCKPIT_PID:-}" ]]; then
      pids+=("${COCKPIT_PID}")
    fi
    if [[ -n "${DENO_PID:-}" ]]; then
      pids+=("${DENO_PID}")
    fi

    if ((${#pids[@]} == 0)); then
      return "${status}"
    fi

    if ! wait -n "${pids[@]}"; then
      status=$?
    else
      status=0
    fi

    local cockpit_alive=1
    if [[ -z "${COCKPIT_PID}" ]] || ! kill -0 "${COCKPIT_PID}" >/dev/null 2>&1; then
      cockpit_alive=0
    fi

    local frontend_alive=1
    if [[ -z "${DENO_PID}" ]] || ! kill -0 "${DENO_PID}" >/dev/null 2>&1; then
      frontend_alive=0
    fi

    if (( cockpit_alive == 0 || frontend_alive == 0 )); then
      if (( cockpit_alive == 0 )); then
        echo "Pilot cockpit backend exited (status ${status})."
        if (( frontend_alive == 1 )); then
          echo "Stopping Pilot frontend..."
          kill "${DENO_PID}" 2>/dev/null || true
        fi
      fi

      if (( frontend_alive == 0 )); then
        echo "Pilot frontend exited (status ${status})."
        if (( cockpit_alive == 1 )); then
          echo "Stopping Pilot cockpit backend..."
          kill "${COCKPIT_PID}" 2>/dev/null || true
        fi
      fi

      if [[ -n "${COCKPIT_PID:-}" ]]; then
        wait "${COCKPIT_PID}" 2>/dev/null || true
      fi
      if [[ -n "${DENO_PID:-}" ]]; then
        wait "${DENO_PID}" 2>/dev/null || true
      fi

      return "${status}"
    fi
  done
}

if ! command -v deno >/dev/null 2>&1; then
  echo "deno is not installed. Install deno before launching the pilot module." >&2
  exit 1
fi
printf '%s after deno check\\n' "$(date -Iseconds)" >> /tmp/pilot_launch_debug.log

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 CLI not found. Source your ROS workspace before launching the pilot module." >&2
  exit 1
fi
printf '%s after ros2 check\\n' "$(date -Iseconds)" >> /tmp/pilot_launch_debug.log

PILOT_PACKAGE_XML="${WORKSPACE_DIR}/install/pilot/share/pilot/package.xml"
if [[ ! -f "${PILOT_PACKAGE_XML}" ]]; then
  echo "pilot ROS package is missing (expected ${PILOT_PACKAGE_XML}). Run 'colcon build --packages-select pilot' before launching." >&2
  exit 1
fi
printf '%s after package check\\n' "$(date -Iseconds)" >> /tmp/pilot_launch_debug.log

echo "Starting Pilot cockpit backend on port ${COCKPIT_PORT}..."
(
  cd "${ROOT_DIR}" &&
    ros2 run pilot cockpit \
      --port "${COCKPIT_PORT}" \
      --log-level "${PILOT_LOG_LEVEL:-info}"
) &
COCKPIT_PID=$!

sleep 2
if ! kill -0 "${COCKPIT_PID}" >/dev/null 2>&1; then
  echo "Pilot cockpit backend failed to start." >&2
  wait "${COCKPIT_PID}" || true
  exit 1
fi

echo "Going to frontend directory: ${FRONTEND_DIR}"
if [[ ! -d "${FRONTEND_DIR}" ]]; then
  echo "Pilot frontend directory is missing (expected ${FRONTEND_DIR})." >&2
  exit 1
fi

if [[ -z "${DENO_TLS_CA_STORE:-}" ]]; then
  # Ensure npm dependencies fetched via deno respect the system trust store.
  # Without this, corporate and lab networks that rely on custom root CAs can
  # cause "invalid peer certificate: UnknownIssuer" failures when the dev
  # server resolves NPM packages on first launch (see workspace handbook).
  export DENO_TLS_CA_STORE=system
fi

echo "Starting Pilot frontend via deno task dev..."
(
  cd "${FRONTEND_DIR}"
  deno task dev
) &
DENO_PID=$!

sleep 2
if ! kill -0 "${DENO_PID}" >/dev/null 2>&1; then
  echo "Pilot frontend failed to start." >&2
  wait "${DENO_PID}" || true
  exit 1
fi

EXIT_CODE=0
monitor_children || EXIT_CODE=$?
exit "${EXIT_CODE}"
