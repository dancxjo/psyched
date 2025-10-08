#!/usr/bin/env bash
# shellcheck shell=bash
#
# Prefix each line from stdin with the module name and mirror it into the log
# file. Optionally forward coloured output to a controlling TTY while the
# original `psh` process is alive. Intended usage:
#
#   exec bash launch.sh \
#     > >(tools/psh/scripts/prefix_logs.sh module stdout module.log "$TTY" "$PSH_PID")

set -euo pipefail

if [[ $# -lt 3 ]]; then
  printf 'usage: %s <module> <stream> <log-file> [tty-path] [caller-pid]\n' "$0" >&2
  exit 64
fi

module_name=$1
stream_name=$2
log_file=$3
tty_path=${4-}
caller_pid=${5-}

case "$stream_name" in
  stdout)
    color_start="\033[2m"
    stream_suffix=""
    ;;
  stderr)
    color_start="\033[31m"
    stream_suffix="[stderr]"
    ;;
  *)
    color_start=""
    stream_suffix=""
    ;;
esac
color_end="\033[0m"
if [[ -z "$color_start" ]]; then
  color_end=""
fi

log_prefix="[$module_name]${stream_suffix:+$stream_suffix}"

# shellcheck disable=SC2094
exec 3>>"$log_file"

tty_active=0

open_tty() {
  if [[ -z "$tty_path" || $tty_active -eq 1 ]]; then
    return
  fi
  if [[ -n "$caller_pid" ]] && ! kill -0 "$caller_pid" 2>/dev/null; then
    tty_path=""
    return
  fi
  # shellcheck disable=SC2094
  if exec 4>>"$tty_path"; then
    tty_active=1
  else
    tty_path=""
  fi
}

close_tty() {
  if [[ $tty_active -eq 1 ]]; then
    exec 4>&-
    tty_active=0
  fi
}

open_tty || true

while IFS= read -r line || [[ -n "$line" ]]; do
  printf '%s %s\n' "$log_prefix" "$line" >&3

  if [[ $tty_active -eq 1 && -n "$caller_pid" ]] && ! kill -0 "$caller_pid" 2>/dev/null; then
    close_tty
    tty_path=""
  fi

  if [[ $tty_active -eq 0 ]]; then
    open_tty || true
  fi

  if [[ $tty_active -eq 1 ]]; then
    if ! printf '%b%s %s%b\n' "$color_start" "$log_prefix" "$line" "$color_end" >&4; then
      close_tty
      tty_path=""
    fi
  fi
done

close_tty
