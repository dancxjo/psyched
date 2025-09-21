# Psyched - ROS2 Multi-Module Robotics System

Psyched is a modular ROS2-based robotics system with host-specific configurations. The system includes modules for audio (ear/voice), vision (eye), navigation (pilot), chat AI, GPS, IMU, and robotic locomotion (foot).

Always reference these instructions first and fallback to search or bash commands only when you encounter unexpected information that does not match the info here.

## CRITICAL Build Limitations

**NETWORK DEPENDENCY FAILURES**: This environment has restricted network access that prevents several critical components from installing:
- ROS2 installation fails due to blocked GitHub API access (api.github.com)
- Chat module fails installing Ollama (ollama.com blocked)
- Voice module fails downloading Piper models (huggingface.co blocked) 
- External git repository clones may fail

**DO NOT attempt the full bootstrap or build process** in environments with network restrictions. Individual module setup can work partially but the complete system requires full network access.

## Working Effectively

### System Architecture
- **Modular Design**: 8 independent modules (chat, ear, eye, foot, gps, imu, pilot, voice)
- **Host-Based Configuration**: Modules are enabled per hostname via symlinks in `hosts/<hostname>/modules/`
- **ROS2 Dependencies**: All modules ultimately require ROS2 "kilted" distro for final build
- **Build System**: Uses colcon with symlink installs for fast development cycles

### Bootstrap and Build Commands (REQUIRE FULL NETWORK ACCESS)
```bash
# NEVER CANCEL: Bootstrap takes 3-5 minutes, may fail on network restrictions
make bootstrap          # timeout: 10+ minutes

# NEVER CANCEL: Build process takes 5-15 minutes depending on modules
make build              # timeout: 30+ minutes

# Module-specific setup (varies by dependencies)
./modules/<module>/setup.sh    # 5 seconds - 2 minutes per module
```

### Individual Module Testing (PARTIAL - some work without full ROS2)
```bash
# Test simple modules (work partially without ROS2):
time ./modules/pilot/setup.sh     # ~5 seconds
time ./modules/ear/setup.sh       # ~30 seconds (installs alsa-utils)

# Test complex modules (fail on network dependencies):
./modules/chat/setup.sh           # fails: requires Ollama download
./modules/voice/setup.sh          # fails: requires Piper models from Hugging Face
./modules/eye/setup.sh            # fails: missing libusb-1.0-dev dependencies
```

### Host Configuration
```bash
# Check available host configurations:
ls hosts/                         # shows: cerebellum

# Test with specific host:
HOST=cerebellum ./tools/setup     # runs setup for all modules in hosts/cerebellum/modules/

# View host module configuration:
ls -la hosts/cerebellum/modules/  # shows symlinks to ../../modules/<name>
```

## Required Dependencies

### System Packages (install before attempting any module setup)
```bash
sudo apt-get update
sudo apt-get install -y \
    build-essential \
    curl \
    make \
    python-is-python3 \
    python3-full \
    python3-pip \
    python3-setuptools \
    python3-dev \
    alsa-utils \
    libusb-1.0-0-dev \
    pkg-config \
    cmake
```

### ROS2 Installation (REQUIRES NETWORK ACCESS)
```bash
# Preferred method (may fail on network restrictions):
make ros2

# Alternative with different distro:
ROS_DISTRO=humble make ros2

# Manual verification:
ls /opt/ros/                      # should show installed ROS2 distros
```

## Module Dependencies and Limitations

### Working Modules (partial setup without ROS2)
- **pilot**: Requires `websockets` Python package (~5 seconds)
- **ear**: Requires `alsa-utils`, `pyaudio` (may fail), `webrtcvad` (~30 seconds)

### Failing Modules (network dependencies)
- **chat**: Requires Ollama installation from ollama.com (BLOCKED)
- **voice**: Requires Piper TTS models from huggingface.co (BLOCKED)  
- **eye**: Requires libfreenect build with libusb-1.0-dev
- **foot/gps/imu**: Require external git repository access (may be blocked)

## Validation Scenarios

**CANNOT BE VALIDATED** in network-restricted environments. In unrestricted environments:

### Complete System Validation
```bash
# 1. Full bootstrap and build (NEVER CANCEL - may take 10+ minutes):
make bootstrap && make build

# 2. Test module launches:
HOST=cerebellum make bringup     # launches all enabled modules in background

# 3. Test pilot web interface (if available):
# Visit http://localhost:8080 for robot control interface
# WebSocket connection on ws://localhost:8081

# 4. Test systemd integration:
make systemd-generate
sudo make systemd-install
sudo make systemd-enable
```

### Individual Module Validation
```bash
# Test ROS2 node launching (requires full ROS2 installation):
source /opt/ros/kilted/setup.bash
source install/setup.bash
ros2 launch pilot pilot.launch.py    # launches pilot web interface
ros2 launch ear ear.launch.py        # launches audio capture
```

## Common Tasks

### Repository Status and Structure
```bash
# Show Makefile targets:
make help

# Repository structure:
ls -la                           # Makefile, packages/, modules/, hosts/, tools/
find packages/ -name "*.py"     # ROS2 Python nodes
find modules/ -name "setup.sh"  # Module setup scripts
```

### Development Workflow
```bash
# 1. Choose host configuration or create new one:
mkdir -p hosts/myhost/modules
ln -s ../../modules/pilot hosts/myhost/modules/pilot

# 2. Setup selected modules:
HOST=myhost ./tools/setup

# 3. Build workspace (requires ROS2):
make build

# 4. Launch modules:
HOST=myhost make bringup
```

### Troubleshooting
```bash
# Check module setup status:
./tools/setup                    # shows host configuration issues

# Debug systemd services (if configured):
make systemd-status
make systemd-debug UNIT=psyched-pilot.service

# Check ROS2 environment:
echo $ROS_DISTRO                 # should show "kilted" or configured distro
source tools/setup_env.sh       # manually source ROS2 + workspace
```

## File Locations

### Key Directories
- `packages/`: ROS2 package definitions with Python nodes
- `modules/`: Module setup scripts and configurations  
- `hosts/`: Host-specific module configurations (symlinks)
- `tools/`: Build and deployment automation scripts
- `src/`: ROS2 workspace source (populated during build)
- `install/`: ROS2 workspace install (created during build)

### Important Files
- `Makefile`: Primary build automation
- `tools/setup_env.sh`: ROS2 environment setup
- `tools/install_ros2.sh`: ROS2 installation script
- `hosts/cerebellum/modules/`: Example complete module configuration

## Expected Timeouts and Limitations

- **Bootstrap**: 3-10 minutes (NEVER CANCEL) - likely to fail on network restrictions
- **Individual module setup**: 5 seconds to 2 minutes each
- **Full build**: 5-15 minutes (NEVER CANCEL) - requires ROS2 installation
- **Network dependencies**: Will fail in restricted environments
- **Hardware dependencies**: Some modules require specific hardware (Kinect, iRobot Create, etc.)

**REMEMBER**: This system is designed for robotics hardware and requires full network access for complete functionality. In restricted environments, focus on code review and structural understanding rather than full system validation.