#!/usr/bin/env bash
# Legacy compatibility shim. Source env/psyched_env.sh instead.

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "workspace_env.sh is deprecated; source env/psyched_env.sh instead." >&2
  exit 1
fi

# shellcheck source=env/psyched_env.sh
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env/psyched_env.sh"
