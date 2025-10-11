# Migration from Deno Fresh to Alpine.js

## Summary

The pilot frontend has been migrated from Deno Fresh (a server-side rendering framework) to simple Alpine.js HTML pages served directly by the cockpit Python backend.

## What Changed

### Removed
- **Deno Fresh frontend** (`modules/pilot/frontend/`) - entire directory removed
- **Module pilot overlays** (`modules/*/pilot/`) - all removed as they contained Fresh components
- **Deno dependencies** from launch scripts
- **backup_frontend** directory and **pilot/pilot** symlinked components

### Added
- **HTTP server** in the Python cockpit bridge (`modules/pilot/packages/pilot/pilot/cockpit/bridge.py`)
  - Serves static files from `modules/pilot/www/`
  - Provides `/api/modules` endpoint that lists active modules from host config
  - Runs on port 8080 by default (configurable via `PILOT_HTTP_PORT`)

- **Alpine.js HTML pages** (`modules/pilot/www/`)
  - `index.html` - main landing page with module grid
  - `modules/imu.html` - IMU telemetry display with quaternion and Euler angles
  - `modules/foot.html` - joystick-based robot control interface
  - `modules/pilot.html` - pilot status page
  - `modules/eye.html` - placeholder for camera controls
  - `modules/ear.html` - placeholder for audio controls

### Modified
- **launch_unit.sh** - simplified to only start the cockpit process
- **shutdown_unit.sh** - simplified to only stop the cockpit process
- **bridge.py** - added HTTP server class and static file serving
- **cli.py** - added `--http-port` argument
- **README.md** files updated to reflect new architecture

## Architecture

```
┌─────────────────┐
│   Browser       │
│  (Alpine.js)    │
└────────┬────────┘
         │ HTTP (port 8080)
         │ WebSocket (port 8088)
         │
┌────────▼────────┐
│  Cockpit Bridge │
│  (Python/ROS)   │
└────────┬────────┘
         │
         │ ROS Topics
         │
┌────────▼────────┐
│   ROS 2 System  │
└─────────────────┘
```

## How It Works

1. The cockpit bridge runs both:
   - A WebSocket server (port 8088) for ROS topic bridging
   - An HTTP server (port 8080) for serving HTML pages

2. HTML pages use Alpine.js for reactivity and connect directly to the WebSocket

3. The cockpit protocol is simple JSON:
   - Subscribe: `{ op: 'sub', topic: '/imu/data' }`
   - Publish: `{ op: 'pub', topic: '/cmd_vel', msg: {...} }`
   - Message: `{ op: 'msg', topic: '/imu/data', msg: {...} }`

4. Active modules are determined from the host config file (`hosts/{hostname}.json`)

## Testing Checklist

After rebuilding the pilot package:

1. **Build the package:**
   ```bash
   colcon build --packages-select pilot
   source install/setup.bash
   ```

2. **Launch the cockpit:**
   ```bash
   psh up pilot
   # or manually:
   ros2 run pilot cockpit --log-level info
   ```

3. **Verify HTTP server:**
   - Open `http://localhost:8080` in a browser
   - Should see the module grid
   - WebSocket status should show "Connected" (green)

4. **Test IMU module:**
   - Click on "imu" module
   - Should load `http://localhost:8080/modules/imu.html`
   - If IMU hardware is connected, should see live telemetry
   - Without hardware, will show "Waiting for IMU data..."

5. **Test Foot control:**
   - Click on "foot" module
   - Should load `http://localhost:8080/modules/foot.html`
   - Joysticks should be draggable
   - If foot hardware is connected, should control the robot
   - Emergency stop button should work

6. **Verify WebSocket communication:**
   - Open browser console (F12)
   - Should see WebSocket messages being logged
   - No connection errors

## Benefits of This Approach

1. **Simpler architecture** - no separate frontend server needed
2. **Direct WebSocket access** - Alpine.js components talk directly to cockpit
3. **Lightweight** - no Node.js, Deno, or complex build pipeline
4. **Faster startup** - single Python process serves everything
5. **Easier debugging** - view source works, no transpilation
6. **Lower resource usage** - one server instead of two

## Future Enhancements

To add a new module page:

1. Create `modules/pilot/www/modules/{module}.html`
2. Copy structure from existing pages (e.g., `imu.html`)
3. Connect to websocket and subscribe to relevant topics
4. Use Alpine.js for reactivity and user interaction

No restart or rebuild needed - just refresh the browser!
