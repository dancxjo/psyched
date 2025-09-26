#!/bin/bash
set -euo pipefail
find src -type f \( -name '*.cpp' -o -name '*.hpp' -o -name '*.h' -o -name '*.cc' -o -name '*.cxx' \) | while read -r file; do
  if grep -Eq 'cv_bridge/cv_bridge\.h([^p]|$)|cv_bridge/cv_bridge\.hppp' "$file"; then
    # Use a Python one-liner so we can precisely normalise the header include.
    # 1. Collapse any previous over-replacement ("hpppp" → "hpp").
    # 2. Replace legacy "cv_bridge.h" includes that are not already suffixed with
    #    an extra "p". Using a lookahead avoids repeatedly appending "pp" when
    #    the setup script runs more than once.
    python3 - "$file" <<'PY'
import pathlib
import re
import sys

path = pathlib.Path(sys.argv[1])
text = path.read_text()
normalised = re.sub(r"cv_bridge/cv_bridge\.hppp+", "cv_bridge/cv_bridge.hpp", text)
normalised = re.sub(r"cv_bridge/cv_bridge\.h(?=[^p]|$)", "cv_bridge/cv_bridge.hpp", normalised)
if normalised != text:
    path.write_text(normalised)
PY
  fi
done
