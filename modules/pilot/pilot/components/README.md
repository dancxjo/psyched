# Pilot Shared Components

This directory contains **shared components** that can be used by multiple modules in the Psyched pilot interface.

## Components in This Directory

These are reusable UI components that are not specific to any single module:

- **pilot-app.js** - Main application shell that hosts all module dashboards
- **pilot-style.js** - Shared CSS styles used across all dashboards
- **pilot-dashboard.js** - The pilot module's own dashboard
- **module-log-viewer.js** - Generic log viewer for any module
- **event-log.js** - Generic event display component
- **diagnostics-panel.js** - Generic diagnostics display
- **value-gauge.js** - Generic gauge widget for numeric values
- **joystick-control.js** - Reusable joystick control for robot motion

## How Symlinking Works

Components in this directory (and all other `modules/*/pilot/components/` directories) are automatically symlinked to the central location at:
```
modules/pilot/packages/pilot/pilot/frontend/components/
```

This symlinking is performed by the `psh mod setup` command (see `tools/psh/lib/module.ts`, function `linkPilotAssets`).

## Creating New Shared Components

If you need to create a new shared component that will be used by multiple modules:

1. Add the component file to this directory (`modules/pilot/pilot/components/`)
2. Run `psh mod setup pilot` to create the symlink
3. Import the component using an absolute path: `/components/your-component.js`

## Creating Module-Specific Components

If you're creating a component for a specific module (e.g., foot, imu, voice):

1. Add the component to that module's directory: `modules/<module>/pilot/components/`
2. Run `psh mod setup <module>` to create the symlink
3. Import shared resources using absolute paths (e.g., `/components/pilot-style.js`)

## Import Conventions

- **Shared components/utilities**: Use absolute paths
  ```javascript
  import { surfaceStyles } from '/components/pilot-style.js';
  import { extractNumeric } from '/utils/metrics.js';
  ```

- **Module-specific utilities**: Use relative paths
  ```javascript
  import { myHelper } from '../utils/my-helper.js';
  ```

## Developer Notes

- Never create files directly in `modules/pilot/packages/pilot/pilot/frontend/components/` - they will be overwritten by symlinks
- The `.gitignore` excludes all `.js` files from the central components directory since they are symlinks
- See `AGENTS.md` in the central location for instructions for AI coding assistants
