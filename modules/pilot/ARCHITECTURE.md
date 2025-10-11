# Pete Cockpit Architecture

## System Overview

```
┌─────────────────────────────────────────────────────────────┐
│                        Browser                              │
│  ┌──────────────────────────────────────────────────────┐  │
│  │              Alpine.js HTML Pages                    │  │
│  │  • index.html (module grid)                          │  │
│  │  • modules/imu.html (telemetry display)              │  │
│  │  • modules/foot.html (joystick controls)             │  │
│  │  • modules/pilot.html, eye.html, ear.html           │  │
│  └────────────────┬───────────────────┬─────────────────┘  │
│                   │                   │                     │
│              HTTP GET              WebSocket                │
│           (static files)         (ROS bridge)              │
└───────────────────┼───────────────────┼─────────────────────┘
                    │                   │
                    │    Port 8080      │    Port 8088
                    │                   │
┌───────────────────▼───────────────────▼─────────────────────┐
│            Python Cockpit Bridge Process                     │
│  ┌──────────────────────┐  ┌─────────────────────────────┐ │
│  │   HTTP Server        │  │   WebSocket Server          │ │
│  │  • Serves /index.html│  │  • Protocol:                │ │
│  │  • Serves /styles.css│  │    - sub: subscribe topic   │ │
│  │  • Serves /modules/* │  │    - pub: publish message   │ │
│  │  • API: /api/modules │  │    - msg: topic update      │ │
│  └──────────────────────┘  └──────────┬──────────────────┘ │
│                                       │                     │
│                            ┌──────────▼──────────────────┐ │
│                            │  ROS Topic Bridge           │ │
│                            │  • /imu/data (subscribe)    │ │
│                            │  • /cmd_vel (publish)       │ │
│                            │  • /conversation (pub/sub)  │ │
│                            │  • /foot/telemetry (sub)    │ │
│                            └──────────┬──────────────────┘ │
└───────────────────────────────────────┼─────────────────────┘
                                        │
┌───────────────────────────────────────▼─────────────────────┐
│                      ROS 2 System                            │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────┐  │
│  │  IMU Module  │  │  Foot Module │  │  Other Modules   │  │
│  │  (hardware)  │  │  (hardware)  │  │  (eye, ear, etc) │  │
│  └──────────────┘  └──────────────┘  └──────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

## Data Flow Examples

### 1. Loading the Cockpit UI

```
Browser                    Cockpit HTTP              File System
   │                           │                          │
   │──── GET / ───────────────>│                          │
   │                           │──── read index.html ────>│
   │                           │<─────────────────────────│
   │<──── index.html ──────────│                          │
   │                           │                          │
   │──── GET /styles.css ─────>│                          │
   │<──── styles.css ──────────│                          │
   │                           │                          │
   │──── GET /api/modules ────>│                          │
   │                     (reads host config)              │
   │<──── JSON ────────────────│                          │
   │  {"modules": ["imu", "foot", ...]}                  │
```

### 2. Viewing IMU Telemetry

```
Browser                WebSocket              ROS Topics
   │                       │                      │
   │─ connect ws:8088/ws ─>│                      │
   │<──── connected ───────│                      │
   │                       │                      │
   │─ {op:"sub",           │                      │
   │   topic:"/imu/data"}─>│                      │
   │                       │── subscribe ────────>│
   │                       │                      │
   │                       │<─ /imu/data msg ─────│
   │<─ {op:"msg",          │                      │
   │    topic:"/imu/data", │                      │
   │    msg:{...}} ────────│                      │
   │                       │                      │
   │  (Alpine.js updates UI)                     │
```

### 3. Controlling the Robot

```
Browser                WebSocket              ROS Topics
   │                       │                      │
   │─ drag joystick ─>     │                      │
   │  (Alpine.js)          │                      │
   │                       │                      │
   │─ {op:"pub",           │                      │
   │   topic:"/cmd_vel",   │                      │
   │   msg:{linear:{...},  │                      │
   │        angular:{...}}}│                      │
   │                      >│── publish ──────────>│
   │                       │                 (robot moves)
   │<─ {op:"ok"} ──────────│                      │
```

## Component Responsibilities

### Browser (Alpine.js)
- **Purpose:** UI rendering and user interaction
- **Technology:** HTML + Alpine.js + vanilla JavaScript
- **Responsibilities:**
  - Display module grid and navigation
  - Render telemetry data (IMU, foot, etc.)
  - Capture user input (joysticks, buttons)
  - Maintain WebSocket connection
  - Handle connection state and errors

### HTTP Server (Python)
- **Purpose:** Serve static assets and API endpoints
- **Technology:** Python asyncio + async TCP server
- **Responsibilities:**
  - Serve HTML, CSS, JavaScript files
  - Provide `/api/modules` endpoint
  - Parse host config to determine active modules
  - Handle MIME types correctly
  - Security: prevent directory traversal

### WebSocket Server (Python)
- **Purpose:** Bridge browser and ROS
- **Technology:** Python websockets library
- **Responsibilities:**
  - Accept WebSocket connections
  - Parse incoming messages (sub/pub operations)
  - Manage topic subscriptions per client
  - Forward ROS messages to browsers
  - Publish browser messages to ROS

### ROS Bridge (Python)
- **Purpose:** Interface with ROS 2 system
- **Technology:** rclpy (ROS Python client)
- **Responsibilities:**
  - Subscribe to ROS topics
  - Publish to ROS topics
  - Convert ROS messages to JSON
  - Convert JSON to ROS messages
  - Manage ROS node lifecycle

## Protocol Specification

### WebSocket Messages

All messages are JSON objects with an `op` field:

#### Subscribe to a topic
```json
{
  "op": "sub",
  "topic": "/imu/data"
}
```

#### Unsubscribe from a topic
```json
{
  "op": "unsub",
  "topic": "/imu/data"
}
```

#### Publish to a topic
```json
{
  "op": "pub",
  "topic": "/cmd_vel",
  "msg": {
    "linear": {"x": 0.5, "y": 0.0, "z": 0.0},
    "angular": {"x": 0.0, "y": 0.0, "z": 0.3}
  }
}
```

#### Message from a topic (server → client)
```json
{
  "op": "msg",
  "topic": "/imu/data",
  "msg": {
    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    "angular_velocity": {"x": 0.01, "y": -0.02, "z": 0.15},
    "linear_acceleration": {"x": 0.05, "y": 0.01, "z": 9.81}
  }
}
```

#### Acknowledgment
```json
{
  "op": "ok"
}
```

#### Error
```json
{
  "op": "err",
  "reason": "unsupported topic '/invalid/topic'"
}
```

## File Structure

```
modules/pilot/
├── packages/pilot/
│   └── pilot/
│       └── cockpit/
│           ├── bridge.py       # Main bridge with HTTP + WS servers
│           ├── cli.py          # Command-line interface
│           ├── protocol.py     # WebSocket protocol handling
│           ├── foot.py         # Foot-specific telemetry
│           ├── status.py       # Status tracking
│           └── image.py        # Image encoding
│
├── www/                        # Static web assets
│   ├── index.html             # Main landing page
│   ├── styles.css             # Shared styles
│   └── modules/               # Module-specific pages
│       ├── imu.html
│       ├── foot.html
│       ├── pilot.html
│       ├── eye.html
│       └── ear.html
│
├── launch_unit.sh             # Launch script
├── shutdown_unit.sh           # Shutdown script
├── module.toml                # Module manifest
├── README.md                  # Module documentation
├── MIGRATION.md               # Migration guide
└── ARCHITECTURE.md            # This file
```

## Adding a New Module Page

To add a new module (e.g., "camera"):

1. **Create HTML file:**
   ```bash
   touch modules/pilot/www/modules/camera.html
   ```

2. **Use template structure:**
   ```html
   <!DOCTYPE html>
   <html lang="en">
   <head>
       <meta charset="UTF-8">
       <title>Camera - Pete Cockpit</title>
       <script defer src="https://cdn.jsdelivr.net/npm/alpinejs@3.x.x/dist/cdn.min.js"></script>
       <link rel="stylesheet" href="/styles.css">
   </head>
   <body>
       <div class="container" x-data="cameraModule()">
           <header>
               <h1>📷 Camera</h1>
               <a href="/" class="back-link">← Back</a>
           </header>
           <!-- Your content here -->
       </div>
       <script>
           function cameraModule() {
               return {
                   ws: null,
                   init() {
                       this.connect();
                   },
                   connect() {
                       const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
                       const wsUrl = `${protocol}//${window.location.hostname}:8088/ws`;
                       this.ws = new WebSocket(wsUrl);
                       this.ws.onopen = () => {
                           // Subscribe to camera topics
                           this.ws.send(JSON.stringify({
                               op: 'sub',
                               topic: '/camera/image_raw'
                           }));
                       };
                       this.ws.onmessage = (event) => {
                           const data = JSON.parse(event.data);
                           // Handle messages
                       };
                   }
               };
           }
       </script>
   </body>
   </html>
   ```

3. **Add to host config** (if needed):
   ```json
   {
     "host": {
       "modules": ["pilot", "imu", "foot", "camera"]
     },
     "modules": {
       "camera": {
         "launch": true
       }
     }
   }
   ```

4. **Restart cockpit** (if running):
   ```bash
   psh down pilot
   psh up pilot
   ```

5. **Visit page:**
   - Go to `http://localhost:8080`
   - Click "camera" in the module grid
   - Or directly visit `http://localhost:8080/modules/camera.html`

No rebuild required! Just refresh the browser.
