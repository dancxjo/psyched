"""Command execution helpers for the pilot backend."""

from __future__ import annotations

import asyncio
import os
import re
from pathlib import Path
from typing import Iterable, List, Mapping

ANSI_ESCAPE_PATTERN = re.compile(r"\x1B\[[0-?]*[ -/]*[@-~]")


def strip_ansi_codes(text: str) -> str:
    """Return text with ANSI escape sequences stripped."""

    return ANSI_ESCAPE_PATTERN.sub("", text)


class CommandExecutor:
    """Execute psh commands asynchronously on behalf of the cockpit."""

    def __init__(self, *, repo_root: Path) -> None:
        self._repo_root = Path(repo_root)

    @property
    def repo_root(self) -> Path:
        return self._repo_root

    def _psh_entrypoint(self) -> Path:
        candidates: List[Path] = []
        explicit = os.environ.get("PSH_ENTRYPOINT")
        if explicit:
            candidates.append(Path(explicit))
        candidates.append(self._repo_root / "tools" / "psh" / "main.ts")
        candidates.append(self._repo_root / "psh" / "main.ts")
        for candidate in candidates:
            if candidate.exists():
                return candidate.resolve()
        raise FileNotFoundError("Unable to locate psh/main.ts; set PSH_ENTRYPOINT or REPO_DIR")

    async def run(
        self,
        scope: str,
        module: str,
        command: str,
        args: Iterable[str] | None = None,
    ) -> Mapping[str, object]:
        """Execute a psh command and return structured output."""

        args = list(args or [])
        deno = os.environ.get("DENO", "deno")
        psh_path = self._psh_entrypoint()

        scope_normalized = (scope or "").strip().lower()
        module_arg: List[str] = [module] if module else []

        if scope_normalized == "mod":
            command_args = [
                "run",
                "-A",
                str(psh_path),
                "mod",
                *module_arg,
                command,
                *args,
            ]
        elif scope_normalized == "sys":
            command_args = [
                "run",
                "-A",
                str(psh_path),
                "sys",
                command,
                *module_arg,
                *args,
            ]
        else:
            command_args = [
                "run",
                "-A",
                str(psh_path),
                scope,
                *module_arg,
                command,
                *args,
            ]

        process = await asyncio.create_subprocess_exec(
            deno,
            *command_args,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
            cwd=str(self._repo_root),
        )
        stdout, stderr = await process.communicate()
        stdout_text = stdout.decode()
        stderr_text = stderr.decode()
        return {
            "code": process.returncode,
            "stdout": stdout_text,
            "stderr": stderr_text,
            "stdout_plain": strip_ansi_codes(stdout_text),
            "stderr_plain": strip_ansi_codes(stderr_text),
        }


__all__ = ["CommandExecutor", "strip_ansi_codes"]
