"""Topic subscription behaviour for the pilot frontend."""

from pathlib import Path
import json
import subprocess


JS_PATH = Path(__file__).resolve().parents[1] / 'pilot' / 'static' / 'joystick.js'


def run_node(script: str) -> None:
    result = subprocess.run(['node', '-e', script], capture_output=True, text=True)
    assert result.returncode == 0, f"stdout: {result.stdout}\nstderr: {result.stderr}"


NODE_PREAMBLE = f"""
const joystickPath = {json.dumps(str(JS_PATH))};

function installDom(protocol = 'https:') {{
    global.window = {{
        location: {{ protocol, hostname: 'pilot.local', port: protocol === 'https:' ? '8443' : '8080' }},
        addEventListener: () => {{}},
        matchMedia: () => ({{ matches: false, addEventListener: () => {{}}, addListener: () => {{}} }}),
        setTimeout,
        clearTimeout,
        setInterval,
        clearInterval,
        requestAnimationFrame: (cb) => setTimeout(cb, 16),
        cancelAnimationFrame: (id) => clearTimeout(id),
        __PILOT_SKIP_AUTO_INIT__: true,
    }};
    const attrStub = {{
        getAttribute: () => null,
    }};
    global.document = {{
        addEventListener: (event, cb) => {{ if (event === 'DOMContentLoaded' && typeof cb === 'function') cb(); }},
        getElementById: () => null,
        createElement: () => ({{ getContext: () => ({{}}) }}),
        body: attrStub,
        documentElement: attrStub,
    }};
}}

class RecordingSocket {{
    constructor(url) {{
        this.url = url;
        this.readyState = 0;
        this.listeners = {{}};
        this.closeCount = 0;
        RecordingSocket.instances.push(this);
    }}
    addEventListener(type, cb) {{
        if (!this.listeners[type]) this.listeners[type] = [];
        this.listeners[type].push(cb);
    }}
    close() {{
        this.readyState = 3;
        this.closeCount += 1;
    }}
    send() {{}}
    static reset() {{ RecordingSocket.instances = []; }}
}}
RecordingSocket.instances = [];
"""


def test_subscribe_to_topic_uses_api_host():
    """subscribeToTopic should open a websocket against the api host."""

    script = NODE_PREAMBLE + """
installDom();
global.WebSocket = RecordingSocket;
RecordingSocket.reset();

const { PilotController } = require(joystickPath);
const controller = new PilotController({ deferInit: true });
const handle = controller.subscribeToTopic('/cmd_vel');

if (!handle) {
    throw new Error('subscribeToTopic should return a handle');
}

if (RecordingSocket.instances.length !== 1) {
    throw new Error('subscribeToTopic did not open a WebSocket');
}

const url = RecordingSocket.instances[0].url;
const expected = 'wss://api/subscribe?topic=%2Fcmd_vel';
if (url !== expected) {
    throw new Error('Unexpected topic websocket url: ' + url);
}

if (typeof handle.close !== 'function') {
    throw new Error('subscription handle missing close() method');
}

handle.close();
"""

    run_node(script)


def test_subscribe_to_topic_normalizes_complex_hosts():
    """Hosts with protocols, ports, or paths should resolve to ws(s) endpoints."""

    script = NODE_PREAMBLE + """
installDom('https:');
global.WebSocket = RecordingSocket;
RecordingSocket.reset();

const { PilotController } = require(joystickPath);

window.PILOT_API_HOST = 'https://api.example.com/base/';
const controllerSecure = new PilotController({ deferInit: true });
const secureHandle = controllerSecure.subscribeToTopic('/cmd_vel');
if (!secureHandle) {
    throw new Error('Expected secure handle');
}
if (RecordingSocket.instances.length !== 1) {
    throw new Error('Expected one websocket for secure host');
}
const secureUrl = RecordingSocket.instances[0].url;
const secureExpected = 'wss://api.example.com/base/subscribe?topic=%2Fcmd_vel';
if (secureUrl !== secureExpected) {
    throw new Error('Secure host normalization failed: ' + secureUrl);
}
secureHandle.close();

// HTTP host should downgrade to ws://
RecordingSocket.reset();
window.PILOT_API_HOST = 'http://api.local:8000/ws/';
window.location.protocol = 'http:';
const controllerInsecure = new PilotController({ deferInit: true });
const insecureHandle = controllerInsecure.subscribeToTopic('cmd_vel');
if (!insecureHandle) {
    throw new Error('Expected insecure handle');
}
if (RecordingSocket.instances.length !== 1) {
    throw new Error('Expected one websocket for insecure host');
}
const insecureUrl = RecordingSocket.instances[0].url;
const insecureExpected = 'ws://api.local:8000/ws/subscribe?topic=cmd_vel';
if (insecureUrl !== insecureExpected) {
    throw new Error('Insecure host normalization failed: ' + insecureUrl);
}
insecureHandle.close();

// Trailing slashes should be trimmed cleanly.
RecordingSocket.reset();
window.PILOT_API_HOST = 'api/';
window.location.protocol = 'https:';
const controllerDefault = new PilotController({ deferInit: true });
const defaultHandle = controllerDefault.subscribeToTopic('/twist');
const defaultUrl = RecordingSocket.instances[0].url;
const defaultExpected = 'wss://api/subscribe?topic=%2Ftwist';
if (defaultUrl !== defaultExpected) {
    throw new Error('Default host normalization failed: ' + defaultUrl);
}
defaultHandle.close();
"""

    run_node(script)


def test_ensure_cmd_vel_subscription_reuses_existing_socket():
    """ensureCmdVelSubscription should reuse sockets until the topic changes."""

    script = NODE_PREAMBLE + """
installDom('https:');
global.WebSocket = RecordingSocket;
RecordingSocket.reset();

const { PilotController } = require(joystickPath);
const controller = new PilotController({ deferInit: true });

controller.ensureCmdVelSubscription('/cmd_vel');
if (RecordingSocket.instances.length !== 1) {
    throw new Error('Expected first cmd_vel subscription');
}
const firstSocket = RecordingSocket.instances[0];
if (firstSocket.closeCount !== 0) {
    throw new Error('Socket should not be closed yet');
}

// Second call with same topic should be a no-op.
controller.ensureCmdVelSubscription('/cmd_vel');
if (RecordingSocket.instances.length !== 1) {
    throw new Error('Unexpected socket recreation for same topic');
}
if (firstSocket.closeCount !== 0) {
    throw new Error('Existing socket should remain open');
}

// Changing the topic should close the existing socket and open a new one.
controller.ensureCmdVelSubscription('/cmd_vel_smoothed');
if (RecordingSocket.instances.length !== 2) {
    throw new Error('Expected second socket for new topic');
}
if (firstSocket.closeCount !== 1) {
    throw new Error('Previous socket should be closed when topic changes');
}
"""

    run_node(script)
