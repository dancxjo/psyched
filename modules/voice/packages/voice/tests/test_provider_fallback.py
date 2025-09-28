from __future__ import annotations

from pathlib import Path
import sys
from typing import Any, Callable

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from voice.providers import (
    ProviderUnavailable,
    build_provider_with_fallback,
)


class DummyProvider:
    def __init__(self, engine: str) -> None:
        self.engine = engine


def config_getter_factory(engine: str) -> Callable[[], object]:
    return lambda: {"engine": engine}


def builder(engine: str, *, logger: Any, config_getter: Callable[[], object]) -> DummyProvider:  # noqa: ANN001
    _ = logger, config_getter
    if engine == "broken":
        raise ProviderUnavailable("nope")
    return DummyProvider(engine)


def test_returns_primary_engine_when_available():
    selected_engine, provider = build_provider_with_fallback(
        preferred="websocket",
        fallbacks=["espeak"],
        logger=None,
        config_getter_factory=config_getter_factory,
        builder=builder,
    )

    assert selected_engine == "websocket"
    assert isinstance(provider, DummyProvider)
    assert provider.engine == "websocket"


def test_uses_fallback_when_primary_unavailable():
    selected_engine, provider = build_provider_with_fallback(
        preferred="broken",
        fallbacks=["espeak"],
        logger=None,
        config_getter_factory=config_getter_factory,
        builder=builder,
    )

    assert selected_engine == "espeak"
    assert provider.engine == "espeak"


def test_raises_when_all_engines_fail():
    with pytest.raises(ProviderUnavailable):
        build_provider_with_fallback(
            preferred="broken",
            fallbacks=["broken"],
            logger=None,
            config_getter_factory=config_getter_factory,
            builder=builder,
        )

