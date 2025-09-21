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
- `packages/` – ROS 2 packages (mostly Python). Each subdirectory is its own
  node package.
- `modules/` – Shell scripts that provision and launch packages for a given
  host.
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

Thanks for keeping Psyched healthy! Update this guide whenever you learn
something the next agent should know.
