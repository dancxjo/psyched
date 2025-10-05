#!/usr/bin/env bash
# shellcheck shell=bash
#
# Prefix each line from stdin with the module name and a stream-specific
# colour. Intended to be used inside process substitutions, e.g.:
#   exec bash launch.sh \
#     > >(tools/psh/scripts/prefix_logs.sh module stdout | tee -a module.log)
#

set -euo pipefail

if [[ $# -lt 2 ]]; then
  printf 'usage: %s <module> <stream>\n' "${0}" >&2
  exit 64
fi

module_name=$1
stream_name=$2
shift 2

# ANSI escape codes for dim (stdout) and red (stderr) highlighting.
case "$stream_name" in
  stdout)
    color_start="\033[2m"
    ;;
  stderr)
    color_start="\033[31m"
    ;;
  *)
    color_start=""
    ;;
esac
color_end="\033[0m"

# Mirror stdin to stdout with the module-prefixed annotation while ensuring
# trailing partial lines are flushed.
while IFS= read -r line || [[ -n "$line" ]]; do
  printf '%b[%s] %s%b\n' "$color_start" "$module_name" "$line" "$color_end"
done
