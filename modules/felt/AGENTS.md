# Felt module guidelines

- Add or update tests alongside code changes; run `PYTHONPATH=modules/felt/packages/felt:$PYTHONPATH pytest modules/felt/packages/felt/tests` after touching Python code.
- Favor dependency injection for external services (LLM, rememberd) so tests can remain hermetic.
- Keep launch and shutdown scripts POSIX-friendly and idempotent; preserve `set -euo pipefail` guards.
