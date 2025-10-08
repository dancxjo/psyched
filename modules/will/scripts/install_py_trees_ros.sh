#!/usr/bin/env bash
set -euo pipefail
if ! python3 -c 'import py_trees_ros' >/dev/null 2>&1; then
  python3 -m pip install py_trees_ros --break-system-packages || \
    python3 -m pip install 'git+https://github.com/splintered-reality/py_trees_ros.git' --break-system-packages || true
fi
