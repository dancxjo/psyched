#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
FRONTEND_DIR="${ROOT_DIR}/modules/pilot/frontend"

if ! command -v deno >/dev/null 2>&1; then
  echo "deno is not installed. Install deno before launching the pilot module." >&2
  exit 1
fi

cd "${FRONTEND_DIR}"

echo "Starting Psyched pilot frontend via deno task start..."
exec deno task start
