# `psh sys` Commands - Systemd Integration

The `psh sys` commands manage systemd system services for modules and services in the Psyched stack.

## Overview

`psh sys` creates and manages systemd system service units that allow modules and services to:
- Start automatically at system boot (as early as possible)
- Run as the user who installed them (not root)
- Restart on failure
- Be controlled via `systemctl` commands (with sudo)
- Appear in system logs (`journalctl`)

## Commands

All `psh sys` subcommands accept an optional target list. If you omit the
target (or pass `*`), the command applies to every module and service marked as
enabled for the active host profile. Use `--module` or `--service` to limit the
wildcard to a single type.

### Setup and Teardown

```bash
# Generate systemd unit files for all enabled modules and services
psh sys setup

# Target a specific module by name
psh sys setup <module-name>

# Generate systemd unit file for a service (use --service flag)
psh sys setup --service <service-name>

# Remove systemd unit file
psh sys teardown <target>
psh sys teardown --service <service-name>
```

### Enable and Disable

```bash
# Enable service to start at boot
psh sys enable
psh sys enable <target>
psh sys enable --service <service-name>

# Disable service from starting at boot
psh sys disable
psh sys disable <target>
psh sys disable --service <service-name>
```

### Start and Stop

```bash
# Start service immediately
psh sys up
psh sys up <target>
psh sys up --service <service-name>

# Stop service
psh sys down
psh sys down <target>
psh sys down --service <service-name>
```

### Debug

```bash
# Show systemctl status and recent logs for all enabled units
psh sys debug

# Inspect a single module or service
psh sys debug <target>
psh sys debug --service <service-name>
```

## Technical Details

### System Services

`psh sys` creates **system systemd services** that run as the installing user. This means:
- Service files are stored in `/etc/systemd/system/`
- Services start at boot automatically (no lingering required)
- Services run with the permissions of the user who installed them
- Services use `sudo systemctl` commands
- Services are visible system-wide

### Security Model

Services are created as system services but include `User=` and `Group=` directives to ensure they run as the non-root user who installed them. This provides:
- Automatic startup at boot time
- Better visibility in system logs
- Proper security isolation (not running as root)
- Access to user's environment and permissions

### Verifying Services

After setting up and enabling a service, you can verify it with:

```bash
# Check service status
sudo systemctl status psh-module-<name>.service
sudo systemctl status psh-service-<name>.service

# View logs (no sudo needed for reading logs)
journalctl -u psh-module-<name>.service -f

# List all psh-managed services
systemctl list-units 'psh-*'
```

### Service Files

Module services are named `psh-module-<name>.service` and include:
- `User=` and `Group=` directives to run as the installing user
- Automatic restart on failure
- Environment variables from host configuration
- Working directory set to the module directory
- Proper ordering with network targets
- `WantedBy=multi-user.target` for early boot startup

Service (container) services are named `psh-service-<name>.service` and include:
- `User=` and `Group=` directives to run as the installing user
- Docker Compose integration for container management
- Network and Docker service dependencies
- One-shot type with RemainAfterExit for proper lifecycle
- `WantedBy=multi-user.target` for early boot startup

## Troubleshooting

### Services don't start at boot

**Symptom:** Services work when manually started but don't start after system reboot.

**Solution:** Ensure the service is enabled:
```bash
sudo systemctl is-enabled psh-module-<name>.service
# Should show: enabled
```

If not enabled:
```bash
sudo systemctl enable psh-module-<name>.service
```

### Services not visible in systemctl

**Symptom:** `systemctl list-units` doesn't show psh services.

**Solution:** 
1. Ensure the service file exists:
   ```bash
   ls -la /etc/systemd/system/psh-*
   ```
2. Reload systemd:
   ```bash
   sudo systemctl daemon-reload
   ```
3. Check for errors:
   ```bash
   sudo systemctl status psh-module-<name>.service
   ```

### Permission errors

**Symptom:** `psh sys setup` fails with permission errors.

**Solution:** The commands require sudo access to write to `/etc/systemd/system/`. Ensure your user has sudo privileges:
```bash
sudo -v  # Verify sudo access
```

## System Requirements

- **systemd**: Must have systemd as the init system (standard on most modern Linux distributions)
- **sudo**: User must have sudo privileges to create and manage system services
- **User account**: Services run as the user who installed them, not as root

## Examples

### Setting up a module with systemd

```bash
# Generate unit file (requires sudo)
psh sys setup pilot

# Enable to start at boot
psh sys enable pilot

# Start now
psh sys up pilot

# Check status
sudo systemctl status psh-module-pilot.service

# View logs (no sudo needed)
journalctl -u psh-module-pilot.service -f
```

### Setting up a service with systemd

```bash
# Generate unit file for a containerized service (requires sudo)
psh sys setup --service tts

# Enable and start
psh sys enable --service tts
psh sys up --service tts

# Check status
sudo systemctl status psh-service-tts.service
```

### Removing a service

```bash
# Stop and disable
psh sys down pilot
psh sys disable pilot

# Remove unit file
psh sys teardown pilot
```

## See Also

- [Host Manifests](./host-manifests.md) - Configure modules and services in host files
- [Module Development](../modules/README.md) - Creating new modules
- [Service Development](../services/README.md) - Creating new services
