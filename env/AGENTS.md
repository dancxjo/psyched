# Psyched environment configuration

Scripts in this directory are sourced by login shells and automation.

- Keep files POSIX/Bash compatible for sourcing (`#!/usr/bin/env bash` is optional but add comments explaining usage).
- Expose helpers through the `psyched::` namespace so callers can compose automation safely.
- Update `tests/psyched_env_test.sh` when behaviour changes to keep shell helpers covered.
- Avoid noisy output unless explicitly requested (pass `--quiet` options where appropriate).
- Keep heredoc terminators flush-left (or use `<<-`) so sourcing doesn't trigger unterminated heredoc warnings.
