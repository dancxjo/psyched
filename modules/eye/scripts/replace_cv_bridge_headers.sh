#!/bin/bash
set -euo pipefail
find src -type f \( -name '*.cpp' -o -name '*.hpp' \) | while read -r file; do
  if grep -q 'cv_bridge/cv_bridge.h' "$file"; then
    sed -i 's#cv_bridge/cv_bridge.h#cv_bridge/cv_bridge.hpp#g' "$file"
  fi
done
