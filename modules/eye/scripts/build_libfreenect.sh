#!/bin/bash
set -euo pipefail
if [ -d src/libfreenect ]; then
  mkdir -p src/libfreenect/build
  cd src/libfreenect/build
  cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr -DBUILD_EXAMPLES=OFF -DBUILD_FAKENECT=OFF ..
  make -j"${NPROC:-$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 1)}"
  if command -v sudo >/dev/null 2>&1; then
    sudo make install || true
  else
    make install || true
  fi
fi
