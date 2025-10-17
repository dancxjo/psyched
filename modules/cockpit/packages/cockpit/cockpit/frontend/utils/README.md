This directory contains the canonical frontend utility modules that are served
from the cockpit frontend root (`/utils/<name>.js`).

Guidelines:
- Put shared helper utilities here (navigation, metrics, audio, voice, battery).
- Modules may keep module-local `cockpit/utils/` helpers, but prefer centralizing
  widely-used helpers here so they survive workspace cleans and are available
  at `/utils/*` without requiring `psh mod setup` symlinks.
- For dynamic sharing at runtime, use the registry helper `registry.js`:
  ```js
  import { register, get, exportsify } from '/utils/registry.js';
  // register a helper
  register('my-helper', { foo: () => {} });
  // or register the default export:
  exportsify('audio', defaultExport);
  // consumer:
  const audio = get('audio');
  ```

This directory is tracked in git and is preserved by `tools/clean_workspace`.
