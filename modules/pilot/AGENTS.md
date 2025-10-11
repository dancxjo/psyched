# Pilot module guidelines

- Document public Python APIs with type annotations and docstrings that include short usage notes.
- Run `PYTHONPATH=modules/pilot/packages/pilot:$PYTHONPATH pytest modules/pilot/packages/pilot/tests` after modifying backend code in this module.
- Keep the cockpit frontend lightweight: static HTML, CSS, and browser JavaScript only (no bundlers or frameworks that require a build step).
- Ensure HTTP endpoints have corresponding unit tests for their request/response contracts where practical.
