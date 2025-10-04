# Ear module guidelines

- Provide comprehensive type annotations and doctrings for public APIs.
- Run `PYTHONPATH=modules/ear/packages/ear:$PYTHONPATH pytest modules/ear/packages/ear/tests` after modifying Python code in this module.
- Prefer backend-agnostic abstractions so additional transcription backends can plug in easily.
- Keep shell scripts POSIX-friendly with `#!/usr/bin/env bash` and `set -euo pipefail`.
