# DO NOT CREATE NEW FILES IN THIS DIRECTORY

This directory (`modules/cockpit/packages/cockpit/cockpit/frontend/components/`) is **SYMLINKS ONLY**.

All component files in this directory are automatically symlinked from module-specific `cockpit/` directories by the `psh mod setup` command.

## How it works

1. Each module (e.g., `modules/foot/`, `modules/imu/`, etc.) has its own `cockpit/components/` directory
2. The `psh mod setup` command creates symlinks from those directories into this central location
3. The cockpit frontend then imports these components using relative paths like `/components/foo-dashboard.js`

## Where to add new components

### Module-specific components
If you're creating a component for a specific module, create it in that module's cockpit directory:
- **Example**: For a foot-related component → `modules/foot/cockpit/components/my-component.js`
- Then run `psh mod setup foot` to create the symlink

### Shared components
For components used by multiple modules, place them in:
- `modules/cockpit/cockpit/components/`
- Then run `psh mod setup cockpit` to create the symlink

## DO NOT:
- ❌ Create new `.js` files directly in this directory
- ❌ Edit symlinked files (always edit the source file in the module directory)
- ❌ Remove this AGENTS.md file

## Reference
See the `linkCockpitAssets` function in `tools/psh/lib/module.ts` for the symlinking implementation.
