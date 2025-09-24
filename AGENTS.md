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
- `modules/` – Module containers and shell scripts that provision and launch
  packages for a given host. Each module should place its ROS2 package(s)
  under `modules/<module>/packages/` so its `setup.sh` can link them into
  the repository `src/` directory before building.
- `hosts/` – Host-specific configuration, symlinks, and generated systemd units.
- `tools/` – Provisioning helpers (ROS installation, bringup orchestration,
  diagnostics, etc.).
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
- Smoke-test launch or setup scripts when you modify them (e.g.
  `./modules/<module>/setup.sh --help`).
- `modules/pilot/packages/pilot/tests/test_ap_node.py` imports `rclpy`; without
  a ROS 2 environment installed run only the targeted tests that avoid it or
  expect collection to fail.

Important workflow note:

- Before running the host/module setup step, remove any existing `./src` so
  module `setup.sh` scripts can recreate it deterministically by symlinking
  module-local packages into `src/`:

```bash
rm -rf ./src
HOST=<your-host> ./tools/setup
make build
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
- `py_trees_ros` is not published to PyPI; `tools/setup_env.sh` installs it
  from GitHub. Ensure network access is available the first time you source the
  environment on a new machine.
- Pilot UI systemd layout no longer renders a `#servicesPills` element; ensure
  frontend updates don't depend on it before processing service data.
- Pilot control cascade reads from DOM elements flagged with `data-source`
  attributes; update `modules/pilot/packages/pilot/tests/test_frontend_layout.py`
  if you adjust that markup.
- Pilot frontend telemetry is bound through an Alpine `$store.pilot` defined in
  `joystick.js`; keep store fields and the `x-text` bindings in
  `static/index.html` in sync when adjusting displayed metrics.
- Topic websocket endpoints are normalised via
  `buildTopicSubscriptionUrl` in `joystick.js`; keep
  `modules/pilot/packages/pilot/tests/test_frontend_topics.py` updated when
  changing host resolution logic.

Thanks for keeping Psyched healthy! Update this guide whenever you learn
something the next agent should know.

## Nav Module: Depth-to-LaserScan + Vision LLM
- RTAB-Map is now a separate module/host; nav module uses AMCL + depth-to-scan pipeline.
- See `nav/depth_projection.py` for depth-to-scan logic and tests.
- See `nav/vision_prompt.py` for vision LLM prompt and annotation stub.
- Depth-scan tests: `tests/test_depth_projection.py`, `tests/test_vision_prompt.py`.
- Update host/module symlinks to configure RTAB-Map separately.
