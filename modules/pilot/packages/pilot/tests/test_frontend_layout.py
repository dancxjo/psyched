"""BDD-style layout expectations for the pilot frontend."""
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent / "pilot" / "static"


def read_html() -> str:
    return (ROOT / "index.html").read_text(encoding="utf-8")


def read_css() -> str:
    return (ROOT / "style.css").read_text(encoding="utf-8")


def test_control_cascade_maps_key_metrics():
    """The top cascade should mirror live control metrics."""
    html = read_html()
    assert 'data-source="linearX"' in html
    assert 'data-source="robotSpeed"' in html
    assert 'data-source="batteryPercentInfo"' in html


def test_dpad_buttons_have_direction_metadata():
    """D-pad removed: joystick should be present instead of discrete D-pad buttons."""
    html = read_html()
    # Ensure the joystick element exists and has expected IDs/classes
    assert 'id="joystick"' in html
    assert 'class="joystick' in html or 'class="joystick-container' in html


def test_css_defines_dpad_and_joystick_styles():
    """The custom stylesheet should render the D-pad cross and joystick."""
    css = read_css()
    # D-pad styles removed; ensure joystick styles remain
    assert ".joystick" in css and ("aspect-ratio" in css or "padding-top" in css)
    assert "@media" in css and "max-width" in css


def test_index_bootstraps_alpine_store():
    """The frontend should expose an Alpine data context for live telemetry."""
    html = read_html()
    assert 'alpinejs' in html.lower()
    assert 'x-data="pilotApp"' in html
    assert '$store.pilot.twist.linearX' in html
