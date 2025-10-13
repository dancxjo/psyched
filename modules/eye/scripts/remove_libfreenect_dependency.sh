#!/usr/bin/env bash
set -euo pipefail

# Ensure the Kinect package does not declare libfreenect as a ROS package dependency.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
SRC_DIR="${PSYCHED_WORKSPACE_SRC:-${REPO_ROOT}/work/src}"
PACKAGE_XML="${SRC_DIR}/kinect_ros2/package.xml"

if [[ ! -f "${PACKAGE_XML}" ]]; then
  echo "[eye/patch] package.xml not found at ${PACKAGE_XML}; skipping"
  exit 0
fi

if ! grep -q '<depend>libfreenect</depend>' "${PACKAGE_XML}"; then
  echo "[eye/patch] libfreenect dependency already absent in ${PACKAGE_XML}"
  exit 0
fi

python3 - "${PACKAGE_XML}" <<'PY'
import pathlib
import sys

path = pathlib.Path(sys.argv[1])
text = path.read_text()
updated = text.replace("\n  <depend>libfreenect</depend>", "")
if updated == text:
    # Fallback to less strict replacement if formatting differs.
    updated = text.replace("<depend>libfreenect</depend>", "")
path.write_text(updated)
PY

echo "[eye/patch] Removed libfreenect dependency from ${PACKAGE_XML}"
