#!/usr/bin/env bash
set -euo pipefail

# Host-specific teardown for pilot module
cd "$(dirname "${BASH_SOURCE[0]}")/../../../modules/pilot"
exec ./teardown.sh "$@"