"""Behavioural checks for the pilot frontend asset layout."""

from __future__ import annotations

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
STATIC_ROOT = PACKAGE_ROOT / "pilot" / "static"


def test_frontend_uses_modular_entrypoint() -> None:
    """The landing page should load the modular app entrypoint."""
    index_html = (STATIC_ROOT / "index.html").read_text(encoding="utf-8")
    assert 'id="pilot-app"' in index_html
    assert 'type="module"' in index_html
    assert "app.js" in index_html


def test_frontend_components_are_split_out() -> None:
    """Expect a dedicated components directory with key modules."""
    components_dir = STATIC_ROOT / "components"
    assert components_dir.is_dir(), "frontend components directory is missing"
    for filename in [
        "pilot-app.js",
        "module-section.js",
        "topic-widget.js",
    ]:
        assert (
            components_dir / filename
        ).exists(), f"expected component module {filename}"


def test_legacy_svelte_sources_are_removed() -> None:
    """Legacy Svelte build artefacts should no longer be committed."""
    assert not (
        PACKAGE_ROOT / "frontend"
    ).exists(), "legacy Svelte sources should be removed after migration"
