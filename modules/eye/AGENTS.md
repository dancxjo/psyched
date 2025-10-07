# Eye module guidelines

- Keep shell scripts POSIX-friendly with `#!/usr/bin/env bash` and `set -euo pipefail`; run `shellcheck modules/eye/**/*.sh` after modifying them.
- Add or update cockpit utilities and front-end code alongside backend changes so operators immediately benefit from telemetry improvements.
- Prefer writing focused unit tests for image encoding utilities; run `pytest modules/pilot/packages/pilot/test/test_image_encoding.py` when touching them.
