# Psyched Agents - Modular Robotics Components

This document provides detailed information about each module (agent) in the Psyched robotics system, their responsibilities, interfaces, and integration patterns.

## Agent Architecture Overview

The Psyched system follows a distributed agent architecture where each module operates as an independent ROS2 node with well-defined interfaces. Agents communicate through ROS2 topics, services, and actions, enabling loose coupling and flexible system composition.

### Core Principles

1. **Autonomy**: Each agent can operate independently when possible
2. **Standardized Interfaces**: Common message types and topic conventions
3. **Graceful Degradation**: System continues operating when individual agents fail
4. **Hot-Swappable**: Agents can be started/stopped without affecting others
5. **Host-Specific Deployment**: Agents are enabled per robot configuration

## Agent Catalog

### 1. Pilot Agent (`pilot`)

**Role**: System coordinator and human-machine interface

**Responsibilities**:
- Web-based robot control interface
- Real-time system monitoring and health reporting
- Joystick/gamepad input translation to robot commands
- System volume and audio control
- Service management interface

**Key Interfaces**:
```yaml
Published Topics:
  /cmd_vel: geometry_msgs/Twist          # Robot movement commands
  /voice/volume: std_msgs/Float32        # Audio volume control

Subscribed Topics:
  /system/health: psyched_msgs/Message   # System health updates

WebSocket API:
  ws://localhost:8081                    # Real-time web interface
  
HTTP Interface:
  http://localhost:8080                  # Web dashboard
```

**Files**:
- Main Node: `packages/pilot/pilot/node.py`
- WebSocket Server: `packages/pilot/pilot/websocket_node.py`
- Web Interface: `packages/pilot/pilot/static/`

**Dependencies**: websockets, ALSA utils

---

### 2. Ear Agent (`ear`)

**Role**: Audio input and preprocessing

**Responsibilities**:
- Microphone audio capture via ALSA
- PCM audio stream publishing
- Voice Activity Detection (VAD)
- Audio preprocessing and noise reduction

**Key Interfaces**:
```yaml
Published Topics:
  /audio/pcm: audio_common_msgs/AudioData   # Raw PCM audio data
  /audio/vad: std_msgs/Bool                 # Voice activity detection

Parameters:
  device: "default"          # ALSA capture device
  rate: 16000               # Sample rate (Hz)
  channels: 1               # Mono/stereo
  chunk: 2048               # Buffer size (bytes)
```

**Audio Pipeline**:
1. ALSA `arecord` captures raw audio
2. PCM data chunked and published
3. VAD analysis for speech detection
4. Configurable preprocessing filters

**Files**:
- Main Node: `packages/ear/ear/node.py`
- Setup Script: `modules/ear/setup.sh`

**Dependencies**: ALSA utils, pyaudio, webrtcvad

---

### 3. Voice Agent (`voice`)

**Role**: Text-to-speech synthesis and audio output

**Responsibilities**:
- Text-to-speech conversion
- Audio playback and volume control
- Multiple TTS engine support
- Voice synthesis coordination with chat agent

**Key Interfaces**:
```yaml
Subscribed Topics:
  /voice: psyched_msgs/Message           # Text to synthesize
  /voice/volume: std_msgs/Float32        # Volume control

Published Topics:
  /voice_done: std_msgs/Empty            # Synthesis completion signal

Parameters:
  engine: "espeak"           # TTS engine (espeak|piper)
  voice_model: "john"        # Voice model selection
  rate: 150                  # Speech rate
  volume: 0.8               # Default volume
```

**TTS Engine Support**:

1. **espeak-ng**: System TTS engine
   - Fast, lightweight
   - Multiple language support
   - MBROLA voice enhancement

2. **Piper**: Neural TTS engine
   - High-quality natural voices
   - Model download from Hugging Face
   - Higher computational requirements

**Voice Coordination**:
- Receives text from chat agent
- Signals completion back to chat
- Prevents conversation overlap

**Files**:
- Main Node: `packages/voice/voice/node.py`
- Setup Script: `modules/voice/setup.sh`

**Dependencies**: espeak-ng or Piper models

---

### 4. Chat Agent (`chat`)

**Role**: Conversational AI interface

**Responsibilities**:
- Natural language conversation processing
- Ollama LLM integration
- Conversation history management
- Voice output coordination

**Key Interfaces**:
```yaml
Subscribed Topics:
  /conversation: psyched_msgs/Message    # Conversation input/output
  /voice_done: std_msgs/Empty           # Voice synthesis completion

Published Topics:
  /conversation: psyched_msgs/Message    # AI responses
  /voice: psyched_msgs/Message          # Text for TTS

Parameters:
  model: "gemma3"                       # Ollama model
  ollama_host: "http://localhost:11434" # Ollama server
  system_prompt: "..."                  # AI personality
  max_history: 20                       # Conversation context
```

**Conversation Flow**:
1. Receives user message on `/conversation`
2. Maintains conversation history with system prompt
3. Queries Ollama LLM for response
4. Sends response to voice agent for TTS
5. Waits for voice completion before continuing
6. Publishes AI response to conversation topic

**Message Format**:
```yaml
role: "user" | "assistant" | "system"
content: "message text"
```

**AI Integration**:
- Supports various Ollama models (Llama, Gemma, etc.)
- Configurable system prompts for personality
- Context-aware conversation with history

**Files**:
- Main Node: `packages/chat/chat/node.py`
- Setup Script: `modules/chat/setup.sh`

**Dependencies**: Ollama, requests library

---

### 5. Eye Agent (`eye`)

**Role**: Computer vision and depth sensing

**Responsibilities**:
- Kinect sensor integration
- RGB and depth image publishing
- Point cloud generation
- Visual object detection and tracking

**Key Interfaces**:
```yaml
Published Topics:
  /camera/rgb/image_raw: sensor_msgs/Image     # RGB camera feed
  /camera/depth/image_raw: sensor_msgs/Image   # Depth camera feed
  /camera/depth/points: sensor_msgs/PointCloud2 # 3D point cloud

Services:
  /eye/calibrate: std_srvs/Empty               # Camera calibration
  /eye/snapshot: std_srvs/Empty                # Capture image

Parameters:
  device_id: 0                                 # Kinect device ID
  fps: 30                                      # Frame rate
  resolution: "640x480"                        # Image resolution
```

**Computer Vision Pipeline**:
1. Kinect sensor initialization via libfreenect
2. RGB and depth frame capture
3. Camera calibration and distortion correction
4. Point cloud generation from depth data
5. Optional object detection and tracking

**Hardware Support**:
- Microsoft Kinect v1
- Microsoft Kinect v2 (via libfreenect2)
- USB 3.0 requirement for Kinect v2

**Files**:
- Setup Script: `modules/eye/setup.sh` (integrates external kinect_ros2)
- External Packages: Clones kinect_ros2 and libfreenect into src/

**Dependencies**: libfreenect, kinect_ros2, OpenCV

---

### 6. Foot Agent (`foot`)

**Role**: Mobile robot base control

**Responsibilities**:
- Differential drive control
- Wheel odometry computation
- Motor safety and limits
- Base sensor integration

**Key Interfaces**:
```yaml
Subscribed Topics:
  /cmd_vel: geometry_msgs/Twist           # Velocity commands

Published Topics:
  /odom: nav_msgs/Odometry               # Wheel odometry
  /foot/status: psyched_msgs/Message      # Motor status

Services:
  /foot/enable: std_srvs/SetBool         # Enable/disable motors
  /foot/reset_odometry: std_srvs/Empty   # Reset position

Parameters:
  serial_port: "/dev/ttyUSB0"            # Robot base connection
  wheel_diameter: 0.072                  # Wheel size (meters)
  wheel_base: 0.235                      # Distance between wheels
  max_linear_vel: 0.5                    # Maximum linear velocity
  max_angular_vel: 2.0                   # Maximum angular velocity
```

**Robot Base Support**:
- iRobot Create/Roomba series
- Generic differential drive robots
- Serial/USB communication protocols

**Safety Features**:
- Velocity limiting and ramping
- Cliff detection integration
- Battery monitoring
- Emergency stop capability

**Files**:
- Setup Script: `modules/foot/setup.sh` (integrates create_robot package)
- External Packages: Clones create_robot and libcreate into src/

**Dependencies**: Serial communication libraries

---

### 7. GPS Agent (`gps`)

**Role**: Global positioning and navigation

**Responsibilities**:
- GNSS receiver integration
- Position and navigation data publishing
- Coordinate system transformations
- Navigation waypoint management

**Key Interfaces**:
```yaml
Published Topics:
  /gps/fix: sensor_msgs/NavSatFix         # GPS position data
  /gps/velocity: geometry_msgs/Vector3Stamped # GPS velocity
  /gps/heading: std_msgs/Float64          # GPS heading

Parameters:
  device: "/dev/ttyACM0"                  # GPS receiver device
  baud_rate: 9600                         # Serial communication rate
  frame_id: "gps_link"                    # TF frame identifier
```

**GNSS Support**:
- u-blox GPS receivers (primary)
- NMEA-compatible receivers
- RTK/DGPS correction support
- Multi-constellation (GPS, GLONASS, Galileo)

**Navigation Integration**:
- ROS Navigation Stack compatible
- Coordinate transformations (WGS84, UTM, local)
- Waypoint following capabilities
- Geofencing support

**Files**:
- Main Node: `packages/ublox_gps/ublox_gps/node.py`
- Setup Script: `modules/gps/setup.sh`

**Dependencies**: u-blox GPS libraries, pyserial

---

### 8. IMU Agent (`imu`)

**Role**: Inertial measurement and orientation

**Responsibilities**:
- Accelerometer, gyroscope, magnetometer data
- Orientation estimation and filtering
- Motion detection and analysis
- Sensor fusion with other localization sources

**Key Interfaces**:
```yaml
Published Topics:
  /imu/data: sensor_msgs/Imu             # Raw IMU data
  /imu/mag: sensor_msgs/MagneticField    # Magnetometer data
  /imu/temperature: sensor_msgs/Temperature # IMU temperature

Parameters:
  device: "/dev/ttyUSB0"                 # IMU device path
  frame_id: "imu_link"                   # TF frame identifier
  rate: 100                              # Publishing rate (Hz)
  calibration_file: "imu_calibration.yaml" # Sensor calibration
```

**IMU Sensor Support**:
- MPU-6050/9250 series
- BNO055 orientation sensor
- Bosch BMI/BMX series
- Custom IMU integrations

**Sensor Fusion**:
- Complementary filtering
- Extended Kalman Filter (EKF)
- Madgwick/Mahony algorithms
- Integration with robot_localization

**Files**:
- Setup Script: `modules/imu/setup.sh` (integrates ros2_mpu6050 package)  
- External Packages: Clones ros2_mpu6050 into src/

**Dependencies**: IMU-specific libraries, sensor drivers

---

## Agent Integration Patterns

### Communication Patterns

1. **Request-Response**: Using ROS2 services for one-time operations
2. **Publish-Subscribe**: Continuous data streams via topics
3. **Action-Based**: Long-running tasks with feedback
4. **Event-Driven**: Trigger-based activations

### Data Flow Examples

#### Voice Conversation Flow
```
User Speech → Ear → [Speech Recognition] → Chat → Ollama → Chat → Voice → Speaker
                                                    ↑               ↓
                                                    └── /voice_done ─┘
```

#### Robot Control Flow
```
Web Interface → Pilot → /cmd_vel → Foot → Robot Base
     ↑                                      ↓
     └────── Status Updates ←─── /odom ────┘
```

#### Sensor Fusion Flow
```
GPS → /gps/fix ┐
              ├→ robot_localization → /robot_pose
IMU → /imu/data ┘
```

### Error Handling and Recovery

1. **Graceful Degradation**: System continues with reduced functionality
2. **Automatic Restart**: Systemd handles agent recovery
3. **Health Monitoring**: Pilot agent tracks system status
4. **Failsafe Modes**: Safe shutdown procedures

### Configuration Management

**Host-Level Configuration**:
- Module selection via symlinks
- Environment variables in config files
- Systemd service generation

**Runtime Parameters**:
- ROS2 parameter server
- Launch file configurations
- Dynamic reconfiguration support

## Development Guidelines

### Adding New Agents

1. **Create Package Structure**:
   ```bash
   mkdir -p packages/myagent/myagent
   # Add setup.py, package.xml, etc.
   ```

2. **Implement ROS2 Node**:
   ```python
   class MyAgentNode(Node):
       def __init__(self):
           super().__init__('myagent_node')
           # Initialize topics, services, parameters
   ```

3. **Create Module Setup**:
   ```bash
   mkdir modules/myagent
   # Add setup.sh, launch.sh, teardown.sh
   ```

4. **Configure Host Integration**:
   ```bash
   ln -s ../../modules/myagent hosts/myhost/modules/
   ```

### Testing Agents

```bash
# Unit Testing
ros2 test packages/myagent

# Integration Testing  
ros2 launch myagent myagent.launch.py
ros2 topic echo /myagent/output

# System Testing
make build
make bringup
```

### Performance Considerations

- **Message Frequency**: Balance data rate vs. system load
- **Queue Sizes**: Prevent message buildup
- **Threading**: Use ROS2 executors appropriately
- **Memory Management**: Clean up resources properly

---

## Summary

The Psyched agent architecture provides a flexible, scalable foundation for robotics applications. Each agent is designed to be autonomous yet cooperative, enabling complex behaviors through simple interactions. The modular design allows for easy customization, testing, and deployment across different robotic platforms.

For implementation details, refer to individual module README files and source code in the `packages/` directory.