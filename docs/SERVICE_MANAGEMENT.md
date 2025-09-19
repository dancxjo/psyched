# Lightweight Cron-Based Service Management

## Overview

Psyched uses a lightweight cron-based service management system instead of traditional systemd services to avoid memory issues and system crashes. This approach provides:

- ✅ **No root privileges required** (unlike systemd)
- ✅ **Minimal memory footprint** (~10-20MB per service vs ~1GB with systemd)
- ✅ **Automatic restart** via cron every minute if services crash
- ✅ **Simple debugging** with accessible log files
- ✅ **Process isolation** to prevent cascading failures
- ✅ **Gradual service startup** to avoid memory spikes

## Why Not Systemd?

Traditional systemd services for ROS2 applications cause problems because:

1. **Heavy ROS2 Environment**: Each service loads the full ROS2 environment, Python virtual environments, and workspace setup
2. **Memory Limits**: Each service can use up to 1GB RAM, and multiple services quickly exhaust system resources
3. **Complex Initialization**: Multiple subprocess spawning during startup creates memory pressure
4. **DDS Middleware Overhead**: ROS2 Cyclone DDS middleware is memory-intensive

## Service Management Commands

### Basic Service Management

```bash
# Install cron-based services for enabled modules (no sudo required!)
make install-services

# Check service status
make status-services

# Stop all services
make stop-services

# Remove all services
make uninstall-services
```

### Individual Service Control

```bash
# Start a specific service manually
make start-service SERVICE=voice

# Stop a specific service
make stop-service SERVICE=voice

# View logs for a service
make logs-service SERVICE=voice LINES=50
```

## Log File Locations

- **Service logs**: `~/.config/psyched/logs/psyched-<module>.log`
- **PID files**: `~/.config/psyched/pids/psyched-<module>.pid`
- **Wrapper scripts**: `~/.config/psyched/wrappers/psyched-<module>`

## How It Works

1. **Cron Monitoring**: Each service has a cron entry that runs every minute
2. **Process Check**: The cron job checks if the service is already running via PID file
3. **Auto-Start**: If not running, the service is automatically started
4. **Lightweight Wrapper**: Each service uses a minimal wrapper script that:
   - Sets up only essential environment variables
   - Sources ROS2 minimally (only if needed)
   - Redirects output to log files
   - Manages PID files for process tracking

## Troubleshooting

### Services Not Starting
```bash
# Check cron entries
crontab -l | grep psyched

# Check logs
make logs-service SERVICE=voice LINES=100

# Try manual start to see errors
make start-service SERVICE=voice
```

### Service Keeps Crashing
```bash
# Check module-specific logs
make logs-service SERVICE=voice

# Try running the launch script directly
cd /path/to/psyched
./modules/voice/launch.sh
```

### Check What Modules Are Enabled
```bash
# List enabled modules for current host
./tools/cron_manager.sh enabled

# List enabled modules for specific host
./tools/cron_manager.sh enabled cerebellum
```

## Complete Setup Example

```bash
# 1. Build the workspace
make build

# 2. Install lightweight cron services  
make install-services

# 3. Wait 1 minute for services to auto-start, then check status
sleep 60
make status-services

# 4. Monitor a specific service
make logs-service SERVICE=voice LINES=20

# 5. Test functionality
make say TEXT="Hello, testing the lightweight services"
```

This lightweight approach provides the same functionality as systemd services but with much lower resource usage and no risk of system crashes due to memory exhaustion.