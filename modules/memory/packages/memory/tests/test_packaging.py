"""Packaging behavior tests ensuring the ROS 2 memory package installs cleanly.

Given the package is installed into an isolated site-packages directory,
When we import the package and inspect the generated console script,
Then the :mod:`memory` runtime is available for ROS 2 to execute.
"""

from __future__ import annotations

import importlib
import subprocess
import sys
import shutil
from pathlib import Path

import pytest


def test_package_installs_and_imports(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """Given the project is installed in isolation, the :mod:`memory` package is importable."""
    package_root = Path(__file__).resolve().parents[1]
    install_root = tmp_path / "site-packages"
    install_root.mkdir()

    egg_info_dir = package_root / f"{package_root.name}.egg-info"
    try:
        subprocess.check_call(
            [
                sys.executable,
                "-m",
                "pip",
                "install",
                "--no-deps",
                "--target",
                str(install_root),
                str(package_root),
            ]
        )

        monkeypatch.syspath_prepend(str(install_root))

        module = importlib.import_module("memory")
        assert module.MemoryService is not None
        entrypoint = install_root / "bin" / "memory_node"
        assert entrypoint.exists(), "Expected the memory_node console script to be installed"
    finally:
        if egg_info_dir.exists():
            shutil.rmtree(egg_info_dir)
