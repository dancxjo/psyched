# No Piper logic present; nothing to remove.


from __future__ import annotations

import importlib
import sys
from pathlib import Path

import pytest


@pytest.fixture(name="utils")
def _utils_module():
    package_root = Path(__file__).resolve().parents[1]
    if str(package_root) not in sys.path:
        sys.path.insert(0, str(package_root))
    module_name = "voice.utils"
    if module_name in sys.modules:
        del sys.modules[module_name]
    return importlib.import_module(module_name)


def test_fetch_fortune_text_success(utils):
    calls = {}

    def fake_which(cmd):
        calls.setdefault("which", []).append(cmd)
        return "/usr/bin/fortune"

    class FakeCompletedProcess:
        def __init__(self):
            self.stdout = "Be excellent to each other.\n"

    def fake_run(args, capture_output, text, timeout):
        calls.setdefault("run", []).append(tuple(args))
        return FakeCompletedProcess()

    text = utils.fetch_fortune_text(which=fake_which, run=fake_run)

    assert text == "Be excellent to each other."
    assert calls["which"] == ["fortune"]
    assert calls["run"] == [("/usr/bin/fortune", "-s")]


def test_fetch_fortune_text_failure(utils):
    def fake_which(_cmd):
        return None

    assert utils.fetch_fortune_text(which=fake_which) is None

    def fake_which_path(_cmd):
        return "/usr/bin/fortune"

    def fake_run(*_args, **_kwargs):
        raise RuntimeError("boom")

    assert utils.fetch_fortune_text(which=fake_which_path, run=fake_run) is None
