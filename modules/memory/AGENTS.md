# Memory module guidelines

- Provide comprehensive type hints, richly documented docstrings, and illustrative usage examples for every public API.
- Follow a TDD loop: add or update tests in `modules/memory/packages/memory/tests` before implementing behaviour.
- Run `PYTHONPATH=modules/memory/packages/memory:$PYTHONPATH pytest modules/memory/packages/memory/tests` after modifying Python code in this module.
- Keep launch scripts POSIX-compliant by starting them with `#!/usr/bin/env bash` and `set -euo pipefail`.
- Record batching or persistence assumptions inline so future maintainers understand the data flow between Qdrant and Neo4j.
- When exporting ROS-specific helpers (like nodes), prefer lazy imports so unit tests can run without ROS dependencies installed.
