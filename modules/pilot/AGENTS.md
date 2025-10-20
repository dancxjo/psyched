# Pilot module guidelines

- Add or update tests alongside code changes; run `PYTHONPATH=modules/pilot/packages/pilot:$PYTHONPATH pytest modules/pilot/packages/pilot/tests` after touching Python code.
- Favor dependency injection for external services (LLM, rememberd) so tests can remain hermetic.
- Keep launch and shutdown scripts POSIX-friendly and idempotent; preserve `set -euo pipefail` guards.
- Keep the cockpit action definitions in `modules/pilot/cockpit/api/` aligned with the node's behaviour so the cockpit web interface, the pilot ROS node, and `psh actions` stay in sync.
- Tests include lightweight stubs for ROS interfaces under `modules/pilot/packages/pilot/tests` so they can run without ROS installed; extend those stubs when adding new dependencies.

## Action discovery

- The canonical action export is available via the `psh` CLI: `psh actions export --json`.
- The pilot node will prefer querying the cockpit HTTP API (`/api/actions`) when available and falls back to reading `modules/<name>/cockpit/api/actions.json` directly when not. Keep the cockpit manifests in sync with runtime behaviour when you add or modify actions.

## Topic configuration

- The pilot automatically subscribes to a minimal set of inputs (host health, `/instant`, `/situation`, `/status`, `/sensations`) so the prompt loop can run even before other modules publish data.
- Modules can suggest extra inputs by dropping `modules/<name>/pilot/topic_suggestions.json`. Entries should include `topic` and `type` fields; placeholders `{HOST}` and `{HOST_SHORT}` resolve at runtime.
- Override the subscriptions at launch time via the `context_topics` and `sensation_topics` parameters (JSON). Example: `ros2 run pilot pilot_node --ros-args -p context_topics='[{"topic":"/hosts/health/motherbrain","type":"psyched_msgs/msg/HostHealth"}]'`.
