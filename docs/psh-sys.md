# `psh sys` Commands - Systemd Integration

The `psh sys` commands manage systemd user services for modules and services in the Psyched stack.

## Overview

`psh sys` creates and manages systemd user service units that allow modules and services to:
- Start automatically at system boot
- Restart on failure
- Be controlled via `systemctl` commands
- Appear in system logs (`journalctl`)

## Commands

### Setup and Teardown

```bash
# Generate systemd unit file for a module
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
psh sys enable <target>
psh sys enable --service <service-name>

# Disable service from starting at boot
psh sys disable <target>
psh sys disable --service <service-name>
```

### Start and Stop

```bash
# Start service immediately
psh sys up <target>
psh sys up --service <service-name>

# Stop service
psh sys down <target>
psh sys down --service <service-name>
```

## Technical Details

### User Services

`psh sys` creates **user systemd services** rather than system services. This means:
- Service files are stored in `~/.config/systemd/user/`
- Services run with user permissions
- Services use `systemctl --user` commands

### Lingering Requirement

For user services to start automatically at boot and remain running across login/logout, **lingering** must be enabled for the user. The `psh sys setup` command automatically enables lingering by running:

```bash
loginctl enable-linger $USER
```

If you see a warning about lingering failing, you may need to enable it manually with sudo:

```bash
sudo loginctl enable-linger $USER
```

### Verifying Services

After setting up and enabling a service, you can verify it with:

```bash
# Check service status
systemctl --user status psh-module-<name>.service
systemctl --user status psh-service-<name>.service

# View logs
journalctl --user -u psh-module-<name>.service -f

# List all psh-managed services
systemctl --user list-units 'psh-*'
```

### Service Files

Module services are named `psh-module-<name>.service` and include:
- Automatic restart on failure
- Environment variables from host configuration
- Working directory set to the module directory
- Proper ordering with network targets

Service (container) services are named `psh-service-<name>.service` and include:
- Docker Compose integration for container management
- Network dependency (waits for network-online.target)
- One-shot type with RemainAfterExit for proper lifecycle

## Troubleshooting

### Services don't start at boot

**Symptom:** Services work when manually started but don't start after system reboot.

**Solution:** Ensure lingering is enabled:
```bash
loginctl show-user $USER | grep Linger
# Should show: Linger=yes
```

If not enabled:
```bash
sudo loginctl enable-linger $USER
```

### Services not visible in systemctl

**Symptom:** `systemctl --user list-units` doesn't show psh services.

**Solution:** 
1. Ensure the service file exists:
   ```bash
   ls -la ~/.config/systemd/user/psh-*
   ```
2. Reload systemd:
   ```bash
   systemctl --user daemon-reload
   ```
3. Check for errors:
   ```bash
   systemctl --user status psh-module-<name>.service
   ```

### Permission errors with loginctl

**Symptom:** `psh sys setup` shows warnings about failing to enable lingering.

**Solution:** This is often not critical. The setup will still work, but you may need to manually enable lingering with sudo:
```bash
sudo loginctl enable-linger $USER
```

## System Requirements

- **systemd**: Must have systemd as the init system (standard on most modern Linux distributions)
- **loginctl**: Must be available for lingering management
- **User session**: Services run in the user session, not as root

## Examples

### Setting up a module with systemd

```bash
# Generate unit file and enable lingering
psh sys setup pilot

# Enable to start at boot
psh sys enable pilot

# Start now
psh sys up pilot

# Check status
systemctl --user status psh-module-pilot.service

# View logs
journalctl --user -u psh-module-pilot.service -f
```

### Setting up a service with systemd

```bash
# Generate unit file for a containerized service
psh sys setup --service tts

# Enable and start
psh sys enable --service tts
psh sys up --service tts

# Check status
systemctl --user status psh-service-tts.service
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
