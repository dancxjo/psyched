# Cockpit Shared Components

This directory contains **shared components** that can be used by multiple modules in the Psyched cockpit interface.

## Components in This Directory

These are reusable UI components that are not specific to any single module:

- **cockpit-app.js** - Main application shell that hosts all module dashboards
- **cockpit-style.js** - Shared CSS styles used across all dashboards
- **cockpit-dashboard.js** - The cockpit module's own dashboard
- **dashboard-collapse.js** - Helper for persistent surface card collapse state
- **module-log-viewer.js** - Generic log viewer for any module
- **event-log.js** - Generic event display component
- **diagnostics-panel.js** - Generic diagnostics display
- **value-gauge.js** - Generic gauge widget for numeric values
- **joystick-control.js** - Reusable joystick control for robot motion

## How Symlinking Works

Components in this directory (and all other `modules/*/cockpit/components/` directories) are automatically symlinked to the central location at:
```
modules/cockpit/packages/cockpit/cockpit/frontend/components/
```

This symlinking is performed by the `psh mod setup` command (see `tools/psh/lib/module.ts`, function `linkCockpitAssets`).

## Creating New Shared Components

If you need to create a new shared component that will be used by multiple modules:

1. Add the component file to this directory (`modules/cockpit/cockpit/components/`)
2. Run `psh mod setup cockpit` to create the symlink
3. Import the component using an absolute path: `/components/your-component.js`

## Creating Module-Specific Components

If you're creating a component for a specific module (e.g., foot, imu, voice):

1. Add the component to that module's directory: `modules/<module>/cockpit/components/`
2. Run `psh mod setup <module>` to create the symlink
3. Import shared resources using absolute paths (e.g., `/components/cockpit-style.js`)

## Import Conventions

- **Shared components/utilities**: Use absolute paths
  ```javascript
  import { surfaceStyles } from '/components/cockpit-style.js';
  import { extractNumeric } from '/utils/metrics.js';
  ```

- **Module-specific utilities**: Use relative paths
  ```javascript
  import { myHelper } from '../utils/my-helper.js';
  ```

## Developer Notes

- Never create files directly in `modules/cockpit/packages/cockpit/cockpit/frontend/components/` - they will be overwritten by symlinks
- The `.gitignore` excludes all `.js` files from the central components directory since they are symlinks
- See `AGENTS.md` in the central location for instructions for AI coding assistants
