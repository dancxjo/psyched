"""Behavioural checks for the pilot frontend asset layout."""

from __future__ import annotations

from pathlib import Path
import re


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
STATIC_ROOT = PACKAGE_ROOT / "pilot" / "static"


def _read_assets() -> str:
    return (STATIC_ROOT / "assets.css").read_text(encoding="utf-8")


def _css_block(css: str, selector: str) -> str:
    pattern = rf"{re.escape(selector)}\s*\{{(.*?)\}}"
    match = re.search(pattern, css, re.DOTALL)
    assert match, f"missing CSS rules for {selector}"
    return match.group(1)


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


def test_control_buttons_share_a_unified_style() -> None:
    """All pilot controls should draw from the shared control button styling."""
    css = _read_assets()
    button_block = _css_block(css, ".control-button")
    assert "border-radius" in button_block
    assert "text-transform" in button_block
    assert "box-shadow" in button_block

    for variant in ["accent", "ghost", "critical"]:
        variant_selector = f'.control-button[data-variant="{variant}"]'
        _css_block(css, variant_selector)


def test_joystick_grid_prevents_touch_scrolling() -> None:
    """The joystick grid should opt-out of browser scrolling gestures."""
    css = _read_assets()
    grid_block = _css_block(css, ".joystick-grid")
    assert "touch-action: none" in grid_block
