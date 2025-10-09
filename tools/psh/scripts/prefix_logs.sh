#!/usr/bin/env bash
# shellcheck shell=bash
#
# Prefix each line from stdin with the module name and a stream-specific note.
# Intended usage:
#
#   exec bash launch.sh \
#     > >(tools/psh/scripts/prefix_logs.sh module stdout | tee -a module.log)

set -euo pipefail

if [[ $# -lt 2 ]]; then
  printf 'usage: %s <module> <stream>\n' "$0" >&2
  exit 64
fi

module_name=$1
stream_name=$2

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

while IFS= read -r line || [[ -n "$line" ]]; do
  printf '%b%s %s%b\n' "$color_start" "$log_prefix" "$line" "$color_end"
done
