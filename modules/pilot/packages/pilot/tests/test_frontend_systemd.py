import json
import subprocess
from pathlib import Path


def test_update_services_list_without_pills():
    """Pilot UI should still render systemd controls when pill container is absent."""
    js_path = Path(__file__).resolve().parents[1] / 'pilot' / 'static' / 'joystick.js'
    script = f"""
const path = require('path');
const joystickPath = {json.dumps(str(js_path))};

global.window = {{
    location: {{ protocol: 'http:', hostname: 'localhost', port: '8080' }},
    addEventListener: () => {{}},
    matchMedia: () => ({{ matches: false, addEventListener: () => {{}}, addListener: () => {{}} }}),
    setTimeout,
    clearTimeout,
    setInterval,
    clearInterval,
    __PILOT_SKIP_AUTO_INIT__: true,
}};

global.document = {{
    addEventListener: (event, cb) => {{ if (typeof cb === 'function') cb(); }},
    getElementById: () => null,
    createElement: () => ({{}}),
}};

global.WebSocket = class {{
    constructor() {{ this.readyState = 1; }}
    send() {{}}
    close() {{}}
}};

const {{ PilotController }} = require(joystickPath);

const controller = new PilotController({{ deferInit: true }});
controller.services = controller.services || {{}};
controller.modules = {{}};
controller.servicesContainer = null;
controller.servicesLogs = {{
    appended: [],
    appendChild(node) {{ this.appended.push(node); return node; }}
}};
controller.serviceDetails = {{}};

const detailCalls = [];
controller.requestSystemdDetail = (unit, lines) => {{ detailCalls.push([unit, lines]); }};
const watchCalls = [];
controller.watchSystemdUnit = (unit, lines) => {{ watchCalls.push([unit, lines]); }};
const controlUpdates = [];
controller.updateServiceLogControls = (svc) => {{ controlUpdates.push(svc); }};
controller.renderServiceLogBlock = (unit) => ({{ unit }});

controller.updateServicesList([{{
    name: 'psyched-nav.service',
    active: 'active',
    enabled: 'enabled',
    status: 'Active: active (running)',
    journal: 'log line',
}}]);

if (!controller.services['psyched-nav.service']) {{
    throw new Error('service not stored');
}}
if (controller.servicesLogs.appended.length !== 1) {{
    throw new Error('service log block not appended');
}}
if (detailCalls.length !== 1 || detailCalls[0][0] !== 'psyched-nav.service') {{
    throw new Error('detail not requested');
}}
if (watchCalls.length !== 1 || watchCalls[0][0] !== 'psyched-nav.service') {{
    throw new Error('watch not requested');
}}
if (controlUpdates.length !== 1 || controlUpdates[0].name !== 'psyched-nav.service') {{
    throw new Error('controls not updated');
}}
if (!controller.serviceDetails['psyched-nav.service']) {{
    throw new Error('service detail cache missing');
}}
"""

    result = subprocess.run(['node', '-e', script], capture_output=True, text=True)
    assert result.returncode == 0, f"stdout: {result.stdout}\nstderr: {result.stderr}"
