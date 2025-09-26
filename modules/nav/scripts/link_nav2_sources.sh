#!/bin/bash
set -euo pipefail
if [ -n "${NAV2_LOCAL_DIR:-}" ] && [ -d "$NAV2_LOCAL_DIR" ]; then
  mkdir -p src
  ln -sfn "$NAV2_LOCAL_DIR" src/nav2_local
fi
