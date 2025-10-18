Pilot cockpit debug UI

This folder contains the cockpit's web interface for the `pilot` module. The dashboard exposes additional debugging controls and views:

- Live stream of the module debug feed exposed via the `pilot.debug_stream` cockpit action. The payload mirrors the ROS `/pilot/debug` topic and includes keys such as `status`, `heartbeat`, `config`, `recent_sensations`, `logs`, and `errors`.
- Buttons to request a debug snapshot (`pilot-debug-request`), request config (`pilot-config-request`), clear cockpit logs (`pilot-clear-logs`), and copy config to clipboard.

Integration notes

- The `PilotNode` in `modules/pilot/packages/pilot/pilot/node.py` publishes a JSON snapshot on `/pilot/debug` every 5s. The cockpit registers the `pilot.debug_stream` action which the dashboard consumes instead of wiring directly to ROS topics.
- For static previews the dashboard will still accept window events named `pilot-module-debug`, `pilot-module-log`, `pilot-module-status` and `pilot-module-message`.

How to use

1. Start `pilot` with the ROS environment sourced and the cockpit server running.
2. Open the cockpit `Pilot Integrator` panel. The debug card will show module status, recent inbound messages, config and a tail of logs and errors.
3. Use the "Request debug snapshot" button to ask the module for an immediate snapshot (the cockpit emits a `pilot-debug-request` CustomEvent which your backend or module action can handle).

Notes

- The debug snapshot is intentionally lightweight and does not ship full log files or large vectors; it samples recent sensation records and basic module metadata to keep the UI responsive.
- If `std_msgs` isn't available at import time (tests, static analysis), the debug publisher will be skipped and the dashboard will still function in a local-only preview mode.
