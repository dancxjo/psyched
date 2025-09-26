# Psyched – Agent Guide

Use this handbook as your quick-start reference when contributing to the
workspace.

## Instruction precedence
- System → developer → user → this guide → in-source comments. Resolve conflicts
  in that order.
- Before editing any file, check for a more specific `AGENTS.md` deeper in that
  directory tree and obey its instructions for the covered files.
- Update this guide whenever you uncover a workflow quirk, fragile script, or
  best practice that future agents should remember.

## Repository orientation
- `packages/` – (Legacy) top-level ROS 2 packages. New structure favors
  module-local packages under `modules/<module>/packages/<pkg>`.
- `modules/` – Module containers and metadata that provision and launch
  packages for a given host. Each module should place its ROS2 package(s)
  under `modules/<module>/packages/` so its `link_packages` action can symlink
  them into the repository `src/` directory before building.
- Module setup is now driven by `modules/<name>/module.toml`. Declare module
  actions (package linking, apt/pip installs, shell commands) there so `psh
  setup` can orchestrate work without bespoke shell scripts. When adding new
  actions check `psh/modules.ts` for supported types.
- Prefer referencing shell helpers via the `script` field on `run` actions
  (e.g. `script = "scripts/install_dep.sh"`) and keep those scripts under the
  module directory with executable permissions so `psh` can resolve them.
- `hosts/` – Host-specific configuration, symlinks, and generated systemd units.
- `tools/` – Provisioning helpers (ROS/Docker installers, env shims used by
  `psh`).
- `src/` – Additional workspace packages pulled in during builds.

## Working style expectations
- Restate the task and outline a plan before modifying files.
- Practice BDD/TDD: capture the expected behaviour (tests, specs, or
  reproducible steps) before or alongside implementation.
- Prefer additive commits locally, but keep the final pushed commit message
  short and meaningful.
- Keep dependencies cached when possible—reuse virtual environments, ROS
  installs, or build artefacts rather than reinstalling from scratch.
- Fix warnings as you go to avoid accruing cleanup debt.

## Coding & documentation standards
- Favor clear, type-annotated Python that matches existing patterns in the
  touched package.
- Provide docstrings or inline commentary for non-obvious logic; include a short
  usage example when introducing public APIs.
- Avoid guarding imports with `try/except`. Handle optional dependencies at the
  call site.
- When you discover you skipped documentation earlier, add a reminder here so
  the next agent anticipates it.

## Testing & validation
- Run the most relevant tests or builds for the files you touched. Prefer
  targeted `pytest` or `colcon test` runs to full workspace builds unless
  necessary.
- Capture the commands you executed and report their outcomes in the final
  summary. If you must skip a check, state why.
- Smoke-test module commands when you modify them (e.g.
  `deno run -A psh/main.ts mod <module> launch`).
- Deno-based tests require local CA trust; run them with
  `DENO_TLS_CA_STORE=system` and grant the needed permissions (e.g.
  `--allow-read --allow-write --allow-env`).
- The container image may not ship with the Deno CLI. Install it locally before
  running `deno test` or other workspace scripts that depend on it.

Important workflow note:

- Before running the host/module setup step, remove any existing `./src` so
  module setup actions can recreate it deterministically by symlinking
  module-local packages into `src/`:

```bash
rm -rf ./src
psh provision <your-host>
source tools/setup_env.sh
colcon build --symlink-install --base-paths src
```

This ensures old or stale package copies don't cause colcon build duplicates
or conflicting package sources.

## Git etiquette
- Stay on the default branch unless explicitly instructed otherwise.
- Do not amend existing commits; always add new ones.
- Leave the tree clean before committing: format, lint, and ensure checks pass.

## Documentation & reminders
- Whenever you update top-level documentation (like this guide or `README.md`),
  cross-check for outdated references between them and fix any inconsistencies.
- Record newly discovered gotchas or workflow aids here using concise bullet
  points.
- Keep the guide succinct and focused on actionable advice.
- `pytest` is configured to ignore the `src/` symlink; place new tests under
  `packages/<pkg>/tests` and stub ROS interfaces when running in environments
  without ROS installed.
- When editing `modules/*/module.toml`, ensure the `[pilot]` block declares a
  `regimes` list and avoid duplicate `[[pilot.topics]]` or redundant
  `link_packages` entries—Pilot’s catalog tests will now fail if either rule is
  violated.
- `py_trees_ros` is not published to PyPI; `tools/setup_env.sh` installs it
  from GitHub. Ensure network access is available the first time you source the
  environment on a new machine.
- The legacy Makefile has been removed; rely on `psh` commands and direct
  `colcon` invocations for setup/build/systemd tasks.
- Pilot UI systemd layout no longer renders a `#servicesPills` element; ensure
  frontend updates don't depend on it before processing service data.
- Pilot control cascade reads from DOM elements flagged with `data-source`
  attributes; update `modules/pilot/packages/pilot/tests/test_frontend_layout.py`
  if you adjust that markup.
- Pilot frontend telemetry now flows through the Svelte store in
  `frontend/src/components/PilotApp.svelte`; update the Alpine interop events
  in `static/index.html` when renaming sections.
- Topic websocket creation lives in `PilotApp.svelte`; adjust
  `test_api_routes.py` and `test_topic_manager.py` when changing connection or
  QoS negotiation logic.
- Git ignores any `src/` directories by default; when adding Svelte sources
  under `modules/pilot/packages/pilot/pilot/frontend/src`, use
  `git add -f ...` to stage them.

Thanks for keeping Psyched healthy! Update this guide whenever you learn
something the next agent should know.

## Nav Module: Depth-to-LaserScan + Vision LLM
- RTAB-Map is now a separate module/host; nav module uses AMCL + depth-to-scan pipeline.
- See `nav/depth_projection.py` for depth-to-scan logic and tests.
- See `nav/vision_prompt.py` for vision LLM prompt and annotation stub.
- Depth-scan tests: `tests/test_depth_projection.py`, `tests/test_vision_prompt.py`.
- Update host/module symlinks to configure RTAB-Map separately.
