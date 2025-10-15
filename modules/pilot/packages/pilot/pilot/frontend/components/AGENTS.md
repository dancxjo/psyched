# DO NOT CREATE NEW FILES IN THIS DIRECTORY

This directory (`modules/pilot/packages/pilot/pilot/frontend/components/`) is **SYMLINKS ONLY**.

All component files in this directory are automatically symlinked from module-specific `pilot/` directories by the `psh mod setup` command.

## How it works

1. Each module (e.g., `modules/foot/`, `modules/imu/`, etc.) has its own `pilot/components/` directory
2. The `psh mod setup` command creates symlinks from those directories into this central location
3. The pilot frontend then imports these components using relative paths like `/components/foo-dashboard.js`

## Where to add new components

### Module-specific components
If you're creating a component for a specific module, create it in that module's pilot directory:
- **Example**: For a foot-related component → `modules/foot/pilot/components/my-component.js`
- Then run `psh mod setup foot` to create the symlink

### Shared components
For components used by multiple modules, place them in:
- `modules/pilot/pilot/components/`
- Then run `psh mod setup pilot` to create the symlink

## DO NOT:
- ❌ Create new `.js` files directly in this directory
- ❌ Edit symlinked files (always edit the source file in the module directory)
- ❌ Remove this AGENTS.md file

## Reference
See the `linkPilotAssets` function in `tools/psh/lib/module.ts` for the symlinking implementation.
