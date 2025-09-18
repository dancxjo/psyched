# Psyched - Experimental Robot Framework

An experimental robot framework built on ROS2 using the ament build system.

## Quick Start

### Prerequisites
- Ubuntu 20.04/22.04 or compatible Linux distribution
- sudo privileges for system installation

### Installation and Setup

1. **Install ROS2 Jazzy** (or your preferred distribution):
   ```bash
   make ros2
   ```

2. **Create and setup workspace**:
   ```bash
   make workspace
   ```

3. **Build the workspace**:
   ```bash
   make build
   ```

4. **Source the environment**:
   ```bash
   # Option 1: View source commands
   make env
   
   # Option 2: Use helper script
   source scripts/setup_env.sh
   
   # Option 3: Manual sourcing
   source /opt/ros/jazzy/setup.bash
   source /opt/psyched/install/setup.bash
   ```

### Usage

After sourcing the environment, you can run the example nodes:

```bash
# Run Python node
ros2 run psyched psyched_node.py

# Run C++ node  
ros2 run psyched psyched_cpp_node

# Launch both nodes together
ros2 launch psyched psyched_launch.py
```

## Development

### Makefile Targets

- `make ros2` - Install ROS2 on the system (default: Jazzy)
- `make workspace` - Create workspace at `/opt/psyched` 
- `make env` - Show environment setup commands
- `make build` - Install dependencies and build workspace
- `make clean` - Clean build artifacts

### Environment Variables

- `ROS_DISTRO` - ROS2 distribution (default: jazzy)
- `WORKSPACE_PATH` - Workspace location (default: /opt/psyched)

### Helper Scripts

- `scripts/setup_env.sh` - Source ROS2 and workspace environment
- `scripts/dev.sh` - Development helper (build, test, clean)

## Package Structure

```
psyched/
├── src/                    # ROS2 source packages
│   └── psyched/           # Main psyched package
│       ├── package.xml    # Package metadata
│       ├── CMakeLists.txt # Build configuration
│       ├── src/           # C++ source files
│       ├── scripts/       # Python nodes
│       └── launch/        # Launch files
├── scripts/               # Top-level utility scripts
├── Makefile              # Build automation
└── README.md             # This file
```
