# Conversant module guidelines

- Keep the node responsive: prefer non-blocking logic and short-lived network calls when invoking local LLM helpers.
- `psh mod setup conversant` now provisions a local Ollama daemon (CPU-only) and pulls `CONVERSANT_LOCAL_LLM_MODEL`; override the model via that env var if you need a different lightweight checkpoint.
- Implement thorough docstrings and type annotations for reusable helpers inside `conversant/`.
- Run `PYTHONPATH=modules/conversant/packages/conversant:$PYTHONPATH pytest modules/conversant/packages/conversant/tests` after changing Python code in this module.
- Use `ros2 launch conversant conversant.launch.py` (via `launch_unit.sh`) to start the full turn-taking loop during development.
- When adding new cockpit actions, update `modules/conversant/cockpit/api/actions.json` and document expected parameters.
- Frontend helpers now ship with Node-based tests—run `node --test modules/conversant/cockpit/components/conversation-helpers.test.mjs` after editing conversation dashboard logic.
