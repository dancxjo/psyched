# Pilot module

The pilot module delivers Pete's browser-based cockpit. It consists of:

- **Backend:** `modules/pilot/packages/pilot/pilot_cockpit` is an `ament_python` package that exposes a unified server at `http://0.0.0.0:8088` (configurable via `PILOT_COCKPIT_PORT`) which handles:
  - HTTP requests for static files and API endpoints
  - WebSocket connections at `/ws` for the cockpit bridge
  
  The server is implemented with `rclpy` + `asyncio` + `websockets` and mirrors cockpit messages to ROS topics (`/conversation`, `/cmd_vel`). Telemetry from `/audio/transcript/final`, `/imu/data`, and the Foot drivetrain is fanned out to any subscribed clients.

- **Frontend:** `modules/pilot/www` contains static HTML pages using Alpine.js that connect directly to the cockpit websocket. The HTTP server automatically serves these pages and provides an API endpoint at `/api/modules` that lists active modules from the host configuration.

## Developing

```bash
# Build + install into work/install
colcon build --packages-select pilot
source install/setup.bash

# Run unit tests (pytest via ament)
colcon test --packages-select pilot
colcon test-result --verbose

# Launch the cockpit (websocket bridge + HTTP server)
ros2 run pilot cockpit --log-level info

# Or with custom paths and port (useful for development):
ros2 run pilot cockpit \
  --port 8088 \
  --www-dir /path/to/psyched/modules/pilot/www \
  --hosts-dir /path/to/psyched/hosts \
  --log-level info
```

The backend stores its ROS-specific logic in `pilot_cockpit/bridge.py` and the Foot telemetry state machine in `pilot_cockpit/foot.py`. Unit tests live in `modules/pilot/packages/pilot/test/` and focus on the Foot state machine helpers. The module ships a `typing_extensions` dependency to keep `rclpy` working on Python 3.12 hosts; remember to rerun `psh mod setup pilot` after editing dependencies so they get applied.

## Frontend customization

The cockpit UI is built with vanilla HTML and Alpine.js. Module-specific pages live in `modules/pilot/www/modules/`. Each page:

- Is served from `http://{host}:8088` by the unified server
- Connects to the cockpit websocket at `ws://{host}:8088/ws`
- Subscribes to relevant ROS topics using the `{ op: 'sub', topic: '/topic/name' }` protocol
- Publishes commands using `{ op: 'pub', topic: '/topic/name', msg: {...} }`

To add a new module page, create `www/modules/{module}.html` following the pattern in existing pages.

## Lifecycle scripts

- `launch_unit.sh` sources the workspace and runs `ros2 run pilot cockpit`.
- `shutdown_unit.sh` stops the cockpit process.

The module declares its apt/pip dependencies in `module.toml`. Ensure you run `psh mod setup pilot` after modifying dependencies so hosts pick up the changes.
