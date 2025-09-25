"""Pytest configuration for the pilot package tests."""

from __future__ import annotations

import sys
from pathlib import Path


def pytest_sessionstart(session):  # noqa: D401 - pytest hook signature
    """Ensure the pilot package is importable without installation."""

    package_root = Path(__file__).resolve().parents[1]
    path_str = str(package_root)
    if path_str not in sys.path:
        sys.path.insert(0, path_str)
