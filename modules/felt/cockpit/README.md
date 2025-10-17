Felt cockpit debug UI

This folder contains the cockpit dashboard for the `felt` module. The dashboard now exposes additional debugging controls and views:

- Live subscription to ROS topic `/felt/debug` (std_msgs/String) which should contain a JSON snapshot with keys such as `status`, `heartbeat`, `config`, `recent_sensations`, `logs`, and `errors`.
- Buttons to request a debug snapshot (`felt-debug-request`), request config (`felt-config-request`), clear cockpit logs (`felt-clear-logs`), and copy config to clipboard.

Integration notes

- The `FeltNode` in `modules/felt/packages/felt/felt/node.py` publishes a JSON snapshot on `/felt/debug` every 5s. The cockpit dashboard subscribes to this topic and will surface the JSON as structured debug information.
- If you prefer the cockpit server to provide module snapshots via HTTP actions, implement an action that returns the same shape and the dashboard will still accept window events named `felt-module-debug`, `felt-module-log`, `felt-module-status` and `felt-module-message` (useful for testing in static previews).

How to use

1. Start `felt` with the ROS environment sourced and the cockpit server running.
2. Open the cockpit `Felt Integrator` panel. The debug card will show module status, recent inbound messages, config and a tail of logs and errors.
3. Use the "Request debug snapshot" button to ask the module for an immediate snapshot (the cockpit emits a `felt-debug-request` CustomEvent which your backend or module action can handle).

Notes

- The debug snapshot is intentionally lightweight and does not ship full log files or large vectors; it samples recent sensation records and basic module metadata to keep the UI responsive.
- If `std_msgs` isn't available at import time (tests, static analysis), the debug publisher will be skipped and the dashboard will still function in a local-only preview mode.
