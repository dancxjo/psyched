# Systemd Service Management for Psyched

This document describes how to manage systemd services for the Psyched project modules.

## Overview

The systemd service management system allows you to:
- Install systemd services that run module launch scripts on boot
- Automatically restart failed services
- Manage services based on host configuration
- Easily revoke and reinstall services during updates

## Architecture

### Files
- `tools/systemd/psyched-module.service.template` - Systemd service template
- `tools/manage_services.sh` - Service management script
- `hosts/[hostname]/enabled/` - Configuration for which modules to enable per host
- `modules/[module]/launch.sh` - Module launch scripts

### Service Template
Each service runs with:
- Proper ROS 2 environment sourcing
- Automatic restart on failure
- Resource limits (2GB memory, 200% CPU)
- Security hardening
- Journal logging

## Usage

### Quick Start

1. **Install services for enabled modules**:
   ```bash
   sudo make install-services
   ```

2. **Start all enabled services**:
   ```bash
   sudo make start-services
   ```

3. **Check service status**:
   ```bash
   make status-services
   ```

### Make Targets

| Target | Description | Requires Sudo |
|--------|-------------|---------------|
| `install-services` | Install systemd services for enabled modules | Yes |
| `uninstall-services` | Remove all psyched systemd services | Yes |
| `update-services` | Uninstall then reinstall services | Yes |
| `start-services` | Start all enabled services | Yes |
| `stop-services` | Stop all psyched services | Yes |
| `status-services` | Show status of all services | No |

#### Host-specific Operations
You can specify a different host:
```bash
sudo make install-services HOST=cerebellum
sudo make start-services HOST=forebrain
```

### Direct Script Usage

The `tools/manage_services.sh` script provides more granular control:

#### Individual Module Management
```bash
# Install service for specific module
sudo ./tools/manage_services.sh install voice

# Start specific service
sudo ./tools/manage_services.sh start voice

# Stop specific service
sudo ./tools/manage_services.sh stop voice

# Check status of specific service
./tools/manage_services.sh status voice

# Uninstall specific service
sudo ./tools/manage_services.sh uninstall voice
```

#### Bulk Operations
```bash
# Install all enabled services for current host
sudo ./tools/manage_services.sh install-enabled

# Install all enabled services for specific host
sudo ./tools/manage_services.sh install-enabled cerebellum

# Start all enabled services
sudo ./tools/manage_services.sh start-enabled

# Stop all services
sudo ./tools/manage_services.sh stop-all

# Uninstall all services
sudo ./tools/manage_services.sh uninstall-all
```

#### Information Commands
```bash
# List all psyched services
./tools/manage_services.sh list

# List available modules
./tools/manage_services.sh modules

# List enabled modules for current host
./tools/manage_services.sh enabled

# List enabled modules for specific host
./tools/manage_services.sh enabled cerebellum
```

## Host Configuration

Configure which modules should run on each host by creating files in `hosts/[hostname]/enabled/`:

```
hosts/
├── cerebellum/
│   └── enabled/
│       ├── 01-voice
│       └── 02-foot
└── forebrain/
    └── enabled/
        └── 01-voice
```

The file names can be:
- Just the module name: `voice`
- Prefixed with numbers for ordering: `01-voice`, `02-foot`

## Service Names

Services are created with the pattern `psyched-[module].service`:
- Voice module → `psyched-voice.service`
- Foot module → `psyched-foot.service`

## Typical Workflow

### Initial Setup
```bash
# Build the workspace
make build

# Install and start services
sudo make install-services
sudo make start-services
```

### After Code Updates
```bash
# Pull changes and rebuild
make update
make build

# Update services (removes old, installs new)
sudo make update-services
sudo make start-services
```

### Development Cycle
```bash
# Stop services during development
sudo make stop-services

# Test manually
./modules/voice/launch.sh

# Restart services
sudo make start-services
```

## Monitoring and Debugging

### Check Service Status
```bash
# Overall status
make status-services

# Specific service
systemctl status psyched-voice.service
```

### View Service Logs
```bash
# Live logs
journalctl -u psyched-voice.service -f

# Recent logs
journalctl -u psyched-voice.service -n 100

# All psyched service logs
journalctl -u "psyched-*.service" -f
```

### Manual Service Control
```bash
# Standard systemctl commands work
sudo systemctl start psyched-voice.service
sudo systemctl stop psyched-voice.service
sudo systemctl restart psyched-voice.service
sudo systemctl disable psyched-voice.service
```

## Environment Variables

The service template supports these environment variables:
- `ROS_DOMAIN_ID` - ROS 2 domain ID (default: 0)
- `USER` - User to run services as (auto-detected)

Set them when running the management script:
```bash
ROS_DOMAIN_ID=1 sudo -E make install-services
```

## Security

Services run with security hardening:
- `NoNewPrivileges=true`
- `ProtectSystem=strict`
- `ProtectHome=read-only`
- Limited read-write paths
- Memory and CPU limits

## Troubleshooting

### Service Won't Start
1. Check if the module's launch.sh is executable:
   ```bash
   ls -la modules/voice/launch.sh
   ```

2. Test the launch script manually:
   ```bash
   ./modules/voice/launch.sh
   ```

3. Check ROS environment:
   ```bash
   source tools/setup_env.sh
   ros2 node list
   ```

### Permission Issues
- Ensure you use `sudo` for service management
- Check file permissions in the repository
- Verify the service user has access to necessary directories

### Multiple Hosts
- Ensure the correct enabled files exist in `hosts/[hostname]/enabled/`
- Use `./tools/manage_services.sh enabled [hostname]` to verify configuration

## Integration with Updates

The `make update` target rebuilds the workspace. After updates:
1. Use `sudo make update-services` to refresh service definitions
2. Start services with `sudo make start-services`

This ensures services use the latest code and configuration.