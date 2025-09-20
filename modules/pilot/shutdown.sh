#!/usr/bin/env bash
set -euo pipefail

# Gracefully stop the pilot module's ros2 launch and any leftover servers on its ports
PATTERN="ros2 launch pilot pilot.launch.py"
TIMEOUT=${TIMEOUT:-10}
WEB_PORT=${PILOT_WEB_PORT:-8080}
WS_PORT=${PILOT_WS_PORT:-8081}

kill_with_pattern() {
  local pattern="$1"
  local timeout="$2"

  mapfile -t pids < <(pgrep -f "$pattern" || true)
  if [ ${#pids[@]} -eq 0 ]; then
    echo "[pilot/shutdown] No matching processes found for pattern: $pattern"
  else
    echo "[pilot/shutdown] Stopping ${#pids[@]} process(es) for: $pattern (SIGTERM)"
    for pid in "${pids[@]}"; do
      if [ "$pid" -ne $$ ]; then
        kill -TERM "$pid" 2>/dev/null || true
      fi
    done

    for ((i=0; i<timeout; i++)); do
      sleep 1
      mapfile -t still_alive < <(pgrep -f "$pattern" || true)
      if [ ${#still_alive[@]} -eq 0 ]; then
        echo "[pilot/shutdown] All processes stopped"
        break
      fi
    done

    # If any processes remain after timeout, force kill them.
    # Note: with 'set -u', referencing an unset array is an error; ensure the array is initialized.
    if [ ${#still_alive[@]} -gt 0 ]; then
      echo "[pilot/shutdown] Forcing stop (SIGKILL) for remaining processes"
      for pid in "${still_alive[@]}"; do
        if [ -n "$pid" ]; then
          kill -KILL "$pid" 2>/dev/null || true
        fi
      done
    fi
  fi
}

kill_port() {
  local port="$1"
  # shellcheck disable=SC2009
  pids=$(ss -ltnp 2>/dev/null | awk -v p=":${port}" '$4 ~ p {print $7}' | sed -E 's/.*pid=([0-9]+).*/\1/' | sort -u)
  if [ -n "$pids" ]; then
    echo "[pilot/shutdown] Killing PIDs on port ${port}: $pids"
    # shellcheck disable=SC2086
    kill -TERM $pids 2>/dev/null || true
  else
    echo "[pilot/shutdown] No process listening on port ${port}"
  fi
}

kill_with_pattern "$PATTERN" "$TIMEOUT"
kill_port "$WEB_PORT"
kill_port "$WS_PORT"
