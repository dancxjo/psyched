# Conversant module guidelines

- Keep the node responsive: prefer non-blocking logic and short-lived network calls when invoking local LLM helpers.
- Implement thorough docstrings and type annotations for reusable helpers inside `conversant/`.
- Run `PYTHONPATH=modules/conversant/packages/conversant:$PYTHONPATH pytest modules/conversant/packages/conversant/tests` after changing Python code in this module.
- Use `ros2 launch conversant conversant.launch.py` (via `launch_unit.sh`) to start the full turn-taking loop during development.
- When adding new cockpit actions, update `modules/conversant/cockpit/api/actions.json` and document expected parameters.
