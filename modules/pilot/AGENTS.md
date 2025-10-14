# Pilot module guidelines

- Document public Python APIs with type annotations and docstrings that include short usage notes.
- Run `PYTHONPATH=modules/pilot/packages/pilot:$PYTHONPATH pytest modules/pilot/packages/pilot/tests` after modifying backend code in this module.
- Ensure test environments provide the runtime dependencies listed in `setup.py` (notably `aiohttp` and `PyYAML`) so pytest runs without skipping coverage.
- Keep the cockpit frontend lightweight: static HTML, CSS, and browser JavaScript only (no bundlers or frameworks that require a build step).
- Reuse the shared utilities in `/components/pilot-style.js` for layout, forms, and controls so dashboards stay visually consistent and compact.
- Ensure HTTP endpoints have corresponding unit tests for their request/response contracts where practical.
- When introducing new module dashboards, register the component tag in `pilot/frontend/components/pilot-app.js` so the cockpit can render it.
- Frontend navigation helpers now have Node-based tests—run `node --test modules/pilot/packages/pilot/pilot/frontend/utils/navigation.test.js` after editing them to keep the pilot sidebar aligned with the rendered modules.
- After editing `pilot/frontend/index.html`, run `node --test modules/pilot/packages/pilot/pilot/frontend/index.test.js` to verify the Alpine-bound navigation stays connected to its data scope.
- Keep `launch_unit.sh` as the supervising process: track helper PIDs, use `trap` + `wait` for cleanup, and avoid `exec` when background jobs are active so `psh mod up` doesn't tear down the cockpit as it exits.
