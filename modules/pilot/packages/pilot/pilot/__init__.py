"""Psyched Pilot cockpit backend package."""

from __future__ import annotations

import sys


try:  # pragma: no cover - success when typing_extensions is installed
    import typing_extensions  # noqa: F401
except ModuleNotFoundError:  # pragma: no cover
    from ._vendor import typing_extensions as _typing_extensions

    sys.modules.setdefault("typing_extensions", _typing_extensions)
