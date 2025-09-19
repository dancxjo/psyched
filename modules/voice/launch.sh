#!/usr/bin/env bash
set -euo pipefail

# Launch the voice node. Requires that the workspace is built and env is sourced.

ros2 launch voice voice.launch.py ${@:-}
