#!/bin/bash
set -euo pipefail
if ! id -nG "$USER" | grep -qw dialout; then
  sudo usermod -aG dialout "$USER" || true
fi
