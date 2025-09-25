# Psyched - ROS2 Multi-Module Robotics System

Psyched is a modular, host-configurable ROS2-based robotics system designed for flexible deployment across different hardware configurations. The system provides a comprehensive suite of robotics capabilities including audio processing, computer vision, navigation, AI chat, GPS positioning, IMU sensing, and robotic locomotion.

## Overview

The system is built around a modular architecture with 8 independent modules that can be selectively enabled per hostname. Each module is a complete ROS2 package with its own dependencies, setup scripts, and launch configurations. The modular design allows for easy customization and deployment on different robotic platforms.

### Key Features

- **Modular Architecture**: 8 independent modules that can be mixed and matched
- **Host-Based Configuration**: Module selection via hostname-specific configurations
- **ROS2 Native**: Built on ROS2 "kilted" distribution with proper package structure
- **Systemd Integration**: Production-ready service management
- **Web Interface**: Browser-based control and monitoring via the pilot module
- **Flexible Build System**: Colcon-based with symlink installs for fast development

## System Architecture

### Core Modules

1. **Pilot** (`pilot`) - Web-based control interface with joystick control
2. **Ear** (`ear`) - Audio capture and processing from microphones
3. **Voice** (`voice`) - Text-to-speech synthesis and audio playback
4. **Chat** (`chat`) - AI conversation interface using Ollama
5. **Eye** (`eye`) - Computer vision using Kinect sensors
6. **Foot** (`foot`) - Robotic locomotion control (iRobot Create integration)
7. **GPS** (`gps`) - GPS positioning and navigation
8. **IMU** (`imu`) - Inertial measurement and orientation sensing

### Message Types

The system uses custom message types defined in `psyched_msgs`:
- `Message.msg` - Chat messages with role (system/user/assistant) and content

### Host Configuration

Modules are enabled per hostname using symlinks in `hosts/<hostname>/modules/`:
```
hosts/
├── cerebellum/           # Example host configuration
│   ├── modules/         # Symlinks to enabled modules
│   │   ├── pilot -> ../../modules/pilot
│   │   ├── ear -> ../../modules/ear
│   │   └── ...
│   ├── config/          # Host-specific configuration
│   └── systemd/         # Generated systemd units
```

## Quick Start

### Prerequisites

- Ubuntu 22.04+ (Debian-based system recommended)
- ROS2 (automatically installed during bootstrap)
- Internet connection for initial setup
- Audio hardware (for ear/voice modules)
- Compatible sensors/actuators (for eye/foot/gps/imu modules)

### Installation

1. **Clone the repository:**
1. **Clone the repository:**
   git clone https://github.com/dancxjo/psyched.git
   cd psyched
   ```

2. **Bootstrap the system (first-time setup):**
3. **Configure your host:**
   make bootstrap
   ```
   This installs ROS2, system dependencies, and sets up the basic environment.

3. **Configure your host:**
   ```bash
   # Check available host configurations
   ls hosts/
4. **Prepare and build the workspace:**

    Important: module setup actions create a clean `src/` tree by symlinking
    module-local packages into the repository `src/` directory. To ensure a
    deterministic setup, delete any existing `./src` folder before running the
    setup step so it can be recreated from scratch:

    ```bash
    rm -rf ./src
    HOST=cerebellum ./tools/setup
    make build
    ```

    Each module should place its ROS2 package(s) under
    `modules/<module>/packages/<pkg>` (for example: `modules/ear/packages/ear`).
    The `link_packages` action in `module.toml` symlinks module-local packages into
    `src/` so `colcon build` (invoked by `make build`) will build them.

4. **Build the workspace:**
   ```bash
   make build
   ```

### Basic Usage

1. **Launch all configured modules:**
   ```bash
   make bringup
   ```

2. **Access the web interface:**
   Open http://localhost:8080 in your browser for robot control

3. **Check system status:**
   ```bash
### Pilot Module
- **Purpose**: Web-based robot control interface
- **Features**: Joystick control, system monitoring, volume control
- **URL**: http://localhost:8080 (web interface)
- **WebSocket**: ws://localhost:8081 (real-time communication)
- **Dependencies**: websockets, ALSA utils

### Ear Module  
- **Purpose**: Audio capture and microphone input
- **Features**: PCM audio publishing, VAD (Voice Activity Detection)
- **Topic**: `/audio/pcm` (audio_common_msgs/AudioData)
- **Dependencies**: ALSA utils, pyaudio, webrtcvad

### Voice Module
- **Purpose**: Text-to-speech synthesis and audio playback
- **Features**: Multiple TTS engines (espeak, Piper), volume control
- **Topic**: `/voice` (input), `/voice/volume` (volume control)
- **Dependencies**: espeak-ng or Piper TTS models

### Chat Module
- **Purpose**: AI conversation interface
- **Features**: Ollama integration, conversation history, voice coordination
- **Topics**: `/conversation` (input/output), `/voice` (TTS output)
- **Dependencies**: Ollama, requests library
- **Models**: Supports various Ollama models (default: gemma3)

### Eye Module
- **Purpose**: Computer vision and depth sensing
- **Features**: Kinect integration, RGB and depth data
- **Dependencies**: libfreenect, kinect_ros2
- **Hardware**: Microsoft Kinect sensor

### Foot Module
- **Purpose**: Mobile robot base control
- **Features**: Differential drive, odometry, collision avoidance
- **Topics**: `/cmd_vel` (geometry_msgs/Twist)
- **Hardware**: iRobot Create or compatible base

### GPS Module
- **Purpose**: Global positioning and navigation
- **Features**: NMEA parsing, position publishing
- **Dependencies**: u-blox GPS receivers
- **Topics**: Standard ROS navigation topics

### IMU Module
- **Purpose**: Inertial measurement and orientation
- **Features**: Accelerometer, gyroscope, magnetometer data
- **Topics**: `/imu/data` (sensor_msgs/Imu)
- **Hardware**: Compatible IMU sensors

## Development

### Building Individual Modules

```bash
# Provision modules for the current host
./tools/setup

# Build workspace
make build

# Launch specific module via psh
deno run -A psh/main.ts mod pilot launch
```

### Adding New Modules

1. Create module directory: `modules/mymodule/`
2. Add ROS 2 package(s) under `modules/mymodule/packages/<pkg>`
3. Define module actions and systemd commands in `modules/mymodule/module.toml`
   (use `link_packages`, dependency actions, and `launch_command`/`shutdown_command`).
4. Add the module name to your host configuration in `hosts/<host>.toml` so `psh setup`
   can provision it.

### Testing Changes

```bash
# Rebuild after changes
make build

# Test specific functionality
ros2 topic echo /cmd_vel
ros2 service list
ros2 node list
```

## Production Deployment

### Systemd Services

```bash
# Generate service units for current host
make systemd-generate

# Install and enable services
sudo make systemd-install
make systemd-enable

# Monitor services
make systemd-status
make systemd-debug UNIT=psyched-pilot.service
```

### Configuration Management

Host-specific settings can be configured in:
- `hosts/<hostname>/config/` - Environment variables and settings
- `hosts/<hostname>/modules/` - Enabled modules (symlinks)

## Troubleshooting

### Common Issues

1. **ROS2 not found**: Run `make ros2` to install ROS2
2. **Build failures**: Check that all module dependencies are installed
3. **Network dependencies**: Some modules require internet access for model downloads
4. **Permission errors**: Ensure user has access to audio/USB devices

### Debug Commands

```bash
# Check environment
source tools/setup_env.sh
echo $ROS_DISTRO

# View logs
make systemd-debug UNIT=psyched-<module>.service

# Test module setup
modules/<module>/module.toml

# Check ROS2 connectivity
ros2 node list
ros2 topic list
```

### Module-Specific Troubleshooting

- **Chat**: Ensure Ollama is running and model is downloaded (`ollama pull gemma3`)
- **Voice**: Check ALSA audio devices (`arecord -l`, `aplay -l`)
- **Eye**: Verify Kinect USB connection and libfreenect installation
- **Foot**: Check serial/USB connection to robot base

## Contributing

Internal maintainers should follow the workflow, testing, and git expectations
documented in [AGENTS.md](AGENTS.md).

1. Fork the repository
2. Create a feature branch
3. Make changes following the modular architecture
4. Test with `make build` and module-specific tests
5. Submit a pull request

### Code Style

- Shell scripts: Use `set -euo pipefail` and proper error handling
- Python: Follow ROS2 Python style guidelines
- Documentation: Update module README.md files for changes

## License

Apache License 2.0 - see [LICENSE](LICENSE) file for details.

## Support

For issues and questions:
- Create GitHub issues for bugs and feature requests
- Check existing module README files for specific documentation
- Review systemd logs for runtime issues

---

**Next Steps**: See [AGENTS.md](AGENTS.md) for the contributor workflow, testing
guidance, and git etiquette that keep this repository healthy.
