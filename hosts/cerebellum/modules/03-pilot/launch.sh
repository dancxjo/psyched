#!/usr/bin/env bash
set -euo pipefail

# Host-specific launch for pilot module
cd "$(dirname "${BASH_SOURCE[0]}")/../../../modules/pilot"
exec ./launch.sh "$@"