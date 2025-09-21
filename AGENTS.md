# Psyched – Agent Guide

This handbook replaces the old module catalog. It collects the expectations and helpful reminders for any agent collaborating on this repository.

## Always follow the instruction chain
- System → developer → user → this handbook → source comments. Resolve conflicts in that order.
- Stop immediately if you see a more specific `AGENTS.md` in a subdirectory you are touching—those rules override the sections below for files in their scope.
- Update this file whenever you discover a workflow quirk, trap, or best practice that should not be forgotten.

## Repository orientation
- **`packages/`** – ROS 2 packages (mostly Python). Each subdirectory is its own node package.
- **`modules/`** – Shell scripts for provisioning and launching packages per host.
- **`hosts/`** – Host-specific configuration, symlinks, and systemd units.
- **`tools/`** – Provisioning helpers (e.g., ROS installation, bringup orchestration).
- **`src/`** – Additional workspace packages pulled in during builds.

## Preferred workflow
1. Restate the task and outline a plan before editing files. Keep the plan in mind while you work.
2. Practice BDD/TDD: identify or write tests/specs before or alongside implementation.
3. Use incremental commits locally, but only push the final succinct commit requested by the user.
4. Keep dependencies cached when possible—reuse existing virtual environments or ROS installations rather than reinstalling them.
5. Leave the tree clean: format, lint, and run relevant checks before committing.

## Coding and documentation expectations
- Favor clear, type-annotated Python. Follow existing patterns in the touched package.
- Provide docstrings or inline comments for non-obvious logic. If you add a public API, include a short usage example in the docstring.
- Fix warnings you encounter while working; do not defer them.
- When introducing new behaviours, include unit/integration tests when feasible.
- Avoid wrapping imports in `try/except` blocks—handle optional dependencies at usage sites.

## Testing & validation
- Minimum expectation: run tests or builds that cover the files you changed.
- For Python packages, prefer targeted `pytest` or `colcon test` runs over full workspace builds unless the change requires it.
- If you modify launch or setup scripts, smoke test them (e.g., `./modules/<module>/setup.sh --help`) when practical.
- Document each command you ran in the final report with its outcome.

## Git and review etiquette
- Stay on the default branch; do not create feature branches unless explicitly told.
- Use concise, meaningful commit messages.
- Never amend or squash existing commits—new work always goes into new commits.
- Keep PR descriptions aligned with cumulative changes; reuse existing wording when doing follow-up tasks unless the scope materially changes.

## When you add reminders here
- Capture gotchas you encounter (missing dependencies, fragile scripts, etc.).
- Phrase reminders as actionable guidance for the next agent.
- Keep the document succinct—prefer bullet lists and short sections over prose dumps.

Thanks for keeping Psyched healthy! Update this guide whenever you learn something that future agents should know.
