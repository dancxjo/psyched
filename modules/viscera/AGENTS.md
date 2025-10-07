# Viscera module guidelines

- Provide comprehensive type annotations, descriptive docstrings, and inline examples for each public API.
- Keep feelers decoupled and pure; they should only depend on the provided :class:`SystemState` snapshot.
- Run `PYTHONPATH=modules/viscera/packages/viscera:$PYTHONPATH pytest modules/viscera/packages/viscera/tests` after modifying code in this module.
- Prefer standard library facilities for system metrics and make optional integrations (such as ``psutil``) defensive.
- Record noteworthy heuristics or future follow-ups as comments near the relevant code paths instead of relying on external notes.
