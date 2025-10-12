"""Command execution helpers for the pilot backend."""

from __future__ import annotations

import asyncio
import os
import re
from pathlib import Path
from typing import Iterable, List, Mapping
import shutil

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

        args = [str(item) for item in (args or [])]

        scope_normalized = (scope or "").strip().lower()
        module_arg: List[str] = [module] if module else []

        psh_cli = os.environ.get("PSH_CLI")
        resolved_cli = None
        if psh_cli:
            resolved_cli = shutil.which(psh_cli)
            if resolved_cli is None:
                candidate = Path(psh_cli)
                if candidate.exists():
                    resolved_cli = str(candidate)
        else:
            resolved_cli = shutil.which("psh")
        use_cli = resolved_cli is not None

        if use_cli:
            command_args: List[str] = [resolved_cli]  # type: ignore[list-item]
        else:
            deno = os.environ.get("DENO", "deno")
            psh_path = self._psh_entrypoint()
            command_args = ["run", "-A", str(psh_path)]

        if scope_normalized == "mod":
            command_args.extend(["mod", command, *module_arg, *args])
        elif scope_normalized == "sys":
            command_args.extend(["sys", command, *module_arg, *args])
        else:
            scoped = (scope or "").strip()
            if scoped:
                command_args.append(scoped)
            command_args.extend(module_arg)
            command_args.append(command)
            command_args.extend(args)

        if use_cli:
            executable = command_args[0]
            run_args = command_args[1:]
        else:
            executable = os.environ.get("DENO", "deno")
            run_args = command_args

        process = await asyncio.create_subprocess_exec(
            executable,
            *run_args,
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
