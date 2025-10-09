# Pilot frontend guidelines

- Keep websocket bootstrap logic (`lib/cockpit_url.ts`) and its regression tests (`lib/cockpit_test.ts`) in sync whenever adjusting connection defaults or overrides.
- Run `deno fmt` and the focused test suite (`deno test lib/cockpit_test.ts`) after touching cockpit client utilities whenever Deno is available in the environment.
- Prefer relative imports or re-export shims for module-provided islands so cockpit builds remain portable across different checkout paths.
