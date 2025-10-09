# Pilot frontend guidelines

- Keep websocket bootstrap logic (`lib/cockpit_url.ts`) and its regression tests (`lib/cockpit_test.ts`) in sync whenever adjusting connection defaults or overrides.
- Run `deno fmt` and the focused test suite (`deno test lib/cockpit_test.ts lib/cockpit_signals_test.ts`) after touching cockpit client utilities whenever Deno is available in the environment.
- When wiring npm packages into the Fresh + Vite toolchain, add them to
  `vite.config.ts` `optimizeDeps.include`/`ssr.noExternal` if they resolve deep
  ESM entry points (for example `@preact/signals`). This keeps the dev server
  from 404ing `node_modules/.deno/*` requests during hydration.
