#!/bin/bash
set -euo pipefail
PATTERN="ros2 launch pilot|pilot_node|pilot_websocket_node|host_health"
TIMEOUT=${TIMEOUT:-10}
WEB_PORT=${PILOT_WEB_PORT:-8080}
WS_PORT=${PILOT_WS_PORT:-8081}
mapfile -t pids < <(pgrep -f "$PATTERN" || true)
if [ ${#pids[@]} -gt 0 ]; then
  echo "[pilot/shutdown] Stopping ${#pids[@]} process(es) for: $PATTERN (SIGTERM)"
  for pid in "${pids[@]}"; do
    if [ "$pid" -ne $$ ]; then
      kill -TERM "$pid" 2>/dev/null || true
    fi
  done
  for ((i=0; i<TIMEOUT; i++)); do
    sleep 1
    mapfile -t still_alive < <(pgrep -f "$PATTERN" || true)
    if [ ${#still_alive[@]} -eq 0 ]; then
      echo "[pilot/shutdown] All processes stopped"
      break
    fi
  done
  if [ ${#still_alive[@]} -gt 0 ]; then
    echo "[pilot/shutdown] Forcing stop (SIGKILL) for remaining processes"
    for pid in "${still_alive[@]}"; do
      if [ -n "$pid" ]; then
        kill -KILL "$pid" 2>/dev/null || true
      fi
    done
  fi
else
  echo "[pilot/shutdown] No matching processes found for pattern: $PATTERN"
fi
for port in "$WEB_PORT" "$WS_PORT"; do
  pids_on_port=$(ss -ltnp 2>/dev/null | awk -v p=":${port}" '$4 ~ p {print $7}' | sed -E 's/.*pid=([0-9]+).*/\1/' | sort -u)
  if [ -n "$pids_on_port" ]; then
    echo "[pilot/shutdown] Killing PIDs on port ${port}: $pids_on_port"
    kill -TERM $pids_on_port 2>/dev/null || true
  else
    echo "[pilot/shutdown] No process listening on port ${port}"
  fi
done
