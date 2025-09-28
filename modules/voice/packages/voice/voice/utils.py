# No Piper logic present; nothing to remove.


from __future__ import annotations

import shutil
import subprocess
from typing import Callable, Optional


def _strip_fortune_attribution(text: str) -> str:
    """Remove attribution lines (``-- Name``) and collapse whitespace."""

    parts = []
    for line in text.replace("\r\n", "\n").split("\n"):
        stripped = line.strip()
        if not stripped:
            continue
        if stripped.startswith("--"):
            continue
        parts.append(stripped)
    return " ".join(parts).strip()


def fetch_fortune_text(
    *,
    which: Callable[[str], Optional[str]] = shutil.which,
    run: Callable[..., subprocess.CompletedProcess[str]] = subprocess.run,
) -> str | None:
    """Return a short fortune string if the ``fortune`` binary is available.

    Args:
        which: Callable used to locate the ``fortune`` binary. Defaults to
            :func:`shutil.which`.
        run: Callable used to execute the command. Defaults to
            :func:`subprocess.run`.

    Returns:
        A trimmed fortune string when available, otherwise ``None``.

    Examples:
        >>> class DummyCompleted:
        ...     def __init__(self, stdout: str) -> None:
        ...         self.stdout = stdout
        >>> fetch_fortune_text(
        ...     which=lambda _: "/usr/bin/fortune",
        ...     run=lambda *args, **kwargs: DummyCompleted("Hello\n"),
        ... )
        'Hello'
    """
    try:
        fortune_path = which("fortune")
    except Exception:
        return None
    if not fortune_path:
        return None
    try:
        result = run([fortune_path, "-s"], capture_output=True, text=True, timeout=3)
    except Exception:
        return None
    output = getattr(result, "stdout", "") or ""
    text = output.strip()
    if not text:
        return None
    cleaned = _strip_fortune_attribution(text)
    return cleaned or None
