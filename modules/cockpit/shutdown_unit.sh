#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="${REPO_DIR:-$(cd "${SCRIPT_DIR}/../.." && pwd)}"
PID_DIR="${REPO_DIR}/work/run/cockpit"

terminate_pattern() {
  local pattern="$1"
  if pgrep -f "$pattern" >/dev/null 2>&1; then
    echo "[cockpit/shutdown] Stopping $pattern"
    pkill -TERM -f "$pattern" || true
  fi
  for _ in {1..10}; do
    if ! pgrep -f "$pattern" >/dev/null 2>&1; then
      echo "[cockpit/shutdown] ${pattern} stopped"
      return 0
    fi
    sleep 1
  done
  pkill -KILL -f "$pattern" || true
  echo "[cockpit/shutdown] Forced kill for $pattern"
}

stop_pid_file() {
  local key="$1"
  local pid_file="${PID_DIR}/${key}.pid"
  if [[ ! -f "${pid_file}" ]]; then
    return 0
  fi
  local pid
  pid="$(<"${pid_file}")"
  if [[ -n "${pid}" ]] && kill -0 "${pid}" >/dev/null 2>&1; then
    echo "[cockpit/shutdown] Stopping ${key} (${pid})"
    kill "${pid}" >/dev/null 2>&1 || true
    for _ in {1..10}; do
      if ! kill -0 "${pid}" >/dev/null 2>&1; then
        echo "[cockpit/shutdown] ${key} stopped"
        break
      fi
      sleep 1
    done
    if kill -0 "${pid}" >/dev/null 2>&1; then
      echo "[cockpit/shutdown] Forcing ${key} (${pid})"
      kill -KILL "${pid}" >/dev/null 2>&1 || true
    fi
  fi
  rm -f "${pid_file}"
  rm -f "${PID_DIR}/${key}.log"
}

terminate_pattern "ros2 run cockpit cockpit"
stop_pid_file "rosbridge"
stop_pid_file "tf2_web_republisher"
stop_pid_file "web_video_server"
