# Pilot module guidelines

- Add or update tests alongside code changes; run `PYTHONPATH=modules/pilot/packages/pilot:$PYTHONPATH pytest modules/pilot/packages/pilot/tests` after touching Python code.
- Favor dependency injection for external services (LLM, rememberd) so tests can remain hermetic.
- Keep launch and shutdown scripts POSIX-friendly and idempotent; preserve `set -euo pipefail` guards.
- Keep the cockpit action definitions in `modules/pilot/cockpit/api/` aligned with the node's behaviour so the cockpit web interface, the pilot ROS node, and `psh actions` stay in sync.

## Action discovery

- The canonical action export is available via the `psh` CLI: `psh actions export --json`.
- The pilot node will prefer querying the cockpit HTTP API (`/api/actions`) when available and falls back to `psh actions export --json` when not. Keep the `psh` command in sync with cockpit actions if you add or modify actions.
