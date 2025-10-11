# Cockpit Module Reference

This document describes the topics and controls for each module in the Pete cockpit.

## Module Overview

### IMU Module (`/modules/imu`)
**Purpose:** Display inertial measurement unit telemetry

**Subscribes to:**
- `/imu/data` - Sensor data including orientation (quaternion), angular velocity, and linear acceleration

**Displays:**
- Quaternion (x, y, z, w)
- Euler angles (roll, pitch, yaw) calculated from quaternion
- Angular velocity (x, y, z in rad/s)
- Linear acceleration (x, y, z in m/s²)

**Controls:** None (read-only telemetry display)

---

### Foot Module (`/modules/foot`)
**Purpose:** Control the iRobot Create robot base and display telemetry

**Subscribes to:**
- `/foot/telemetry` - Aggregated telemetry from Create robot including:
  - Battery status (charge %, voltage, current, temperature, charging state)
  - Odometry (wheel encoder velocities)
  - Hazards (bumper and cliff sensor states)
  - Robot status (operating mode, last command time)

**Publishes to:**
- `/cmd_vel` - Movement commands with velocity limits:
  - `linear.x`: -0.5 to 0.5 m/s (forward/backward)
  - `angular.z`: -4.25 to 4.25 rad/s (rotation)

**Controls:**
- Dual joystick interface for linear and angular velocity control
- Emergency stop button

**Displays:**
- Current command velocities
- Battery charge percentage, voltage, current, charging state
- Odometry (actual robot velocity)
- Hazard warnings (bumpers, cliff sensors) with red highlighting when active
- Robot operating mode and last command timestamp

---

### Ear Module (`/modules/ear`)
**Purpose:** Display audio transcription from speech recognition

**Subscribes to:**
- `/audio/transcript/final` - Final transcribed text from speech recognition

**Displays:**
- List of recent transcripts with timestamps (up to 50 most recent)
- Real-time updates as new speech is recognized

**Controls:** None (read-only transcript display)

---

### Eye Module (`/modules/eye`)
**Purpose:** Display camera feeds from robot vision system

**Subscribes to:**
- `/image_raw` - RGB camera feed (JPEG encoded, base64)
- `/camera/depth/image_raw` - Depth camera feed (JPEG encoded, base64)

**Displays:**
- RGB camera view with timestamp
- Depth camera view with timestamp
- "No feed available" message when camera is not publishing

**Controls:** None (read-only video display)

---

### Pilot Module (`/modules/pilot`)
**Purpose:** System status and module overview

**Subscribes to:**
- `/_status` - Internal cockpit status topic showing:
  - Active modules with subscriptions
  - Number of connected clients

**Displays:**
- List of active modules
- Number of connected websocket clients
- System health information

**Controls:** None (status display only)

---

## Adding New Modules

To add a new module page:

1. **Create HTML file:** Add `modules/pilot/www/modules/yourmodule.html`
2. **Subscribe to topics:** Use the websocket protocol:
   ```javascript
   this.send({ op: 'sub', topic: '/your/topic' })
   ```
3. **Publish commands:** Use the websocket protocol:
   ```javascript
   this.send({ op: 'pub', topic: '/your/topic', msg: { ... } })
   ```
4. **Update bridge:** If your topic isn't already supported, add it to `_SIMPLE_TOPIC_RELAYS` or `_MODULE_TOPIC_MAP` in `bridge.py`
5. **Add to host config:** Include your module in the host's `modules` list to make it appear in the cockpit grid

## Websocket Protocol

All modules use the same websocket protocol to communicate with the cockpit bridge:

**Subscribe to a topic:**
```json
{ "op": "sub", "topic": "/topic/name" }
```

**Unsubscribe from a topic:**
```json
{ "op": "unsub", "topic": "/topic/name" }
```

**Publish to a topic:**
```json
{ "op": "pub", "topic": "/topic/name", "msg": { ... } }
```

**Receive messages:**
```json
{ "op": "msg", "topic": "/topic/name", "msg": { ... } }
```

**Response messages:**
```json
{ "op": "ok" }
{ "op": "err", "reason": "error description" }
```
