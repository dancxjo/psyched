# Voice module guidelines

- Prefer type annotations and detailed docstrings for every public class, method, and function.
- Run `PYTHONPATH=modules/voice/packages/voice:$PYTHONPATH pytest modules/voice/packages/voice/tests` after modifying Python code in this module.
- Keep shell scripts POSIX-friendly with `#!/usr/bin/env bash` and `set -euo pipefail`.
