# Pilot module

The pilot module delivers Pete's browser-based cockpit. It is split into two pieces:

- **Backend:** `modules/pilot/packages/pilot/pilot_cockpit` is an `ament_python` package that exposes a websocket bridge at `ws://0.0.0.0:8088/ws` by default. Override the port via the `PILOT_COCKPIT_PORT` environment variable (propagated by the `ros2` service) to keep the frontend and backend aligned. The bridge is implemented with `rclpy` + `asyncio` + `websockets` and mirrors cockpit messages to ROS topics (`/conversation`, `/cmd_vel`). Telemetry from `/audio/transcript/final`, `/imu/data`, and the Foot drivetrain is fanned out to any subscribed clients.
- **Frontend:** `modules/pilot/frontend` is a Deno Fresh application that renders the cockpit UI and consumes the websocket bridge via `lib/cockpit.ts`.

## Developing the backend

```bash
# Build + install into work/install
colcon build --packages-select pilot
source install/setup.bash

# Run unit tests (pytest via ament)
colcon test --packages-select pilot
colcon test-result --verbose

# Launch the websocket bridge
ros2 run pilot cockpit --log-level info
```

The backend stores its ROS-specific logic in `pilot_cockpit/bridge.py` and the Foot telemetry state machine in `pilot_cockpit/foot.py`. Unit tests live in `modules/pilot/packages/pilot/test/` and focus on the Foot state machine helpers. The module ships a `typing_extensions` dependency to keep `rclpy` working on Python 3.12 hosts; remember to rerun `psh mod setup pilot` after editing dependencies so they get applied.

## Developing the frontend

```bash
cd modules/pilot/frontend
# Live reload dev server
deno task dev
# Lint / type-check
deno fmt
deno check lib/cockpit.ts
# Run frontend tests
deno test
```

## Lifecycle scripts

- `launch_unit.sh` sources the workspace, runs `ros2 run pilot cockpit`, and starts the Fresh dev server.
- `shutdown_unit.sh` stops the websocket bridge and the Fresh dev server.

The module declares its apt/pip dependencies in `module.toml`. Ensure you run `psh mod setup pilot` after modifying dependencies so hosts pick up the changes.
