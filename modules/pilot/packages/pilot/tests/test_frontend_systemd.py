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


def test_modules_group_existing_service_blocks():
    """Services discovered before module metadata should regroup under modules once available."""
    js_path = Path(__file__).resolve().parents[1] / 'pilot' / 'static' / 'joystick.js'
    script = f"""
const path = require('path');
const joystickPath = {json.dumps(str(js_path))};

const elements = new Map();
function makeElement(id, classes) {{
    classes = Array.isArray(classes) ? classes : (classes ? String(classes).split(/\s+/).filter(Boolean) : []);
    const classSet = new Set(classes);
    const el = {{
        id: id || '',
        children: [],
        parentElement: null,
        dataset: {{}},
        _classSet: classSet,
        className: Array.from(classSet).join(' '),
        classList: {{
            contains(cls) {{ return classSet.has(cls); }},
            add(cls) {{ classSet.add(cls); el.className = Array.from(classSet).join(' '); }},
            remove(cls) {{ classSet.delete(cls); el.className = Array.from(classSet).join(' '); }},
        }},
        appendChild(node) {{
            if (!node) return node;
            if (node.parentElement && node.parentElement !== el) {{
                node.parentElement.removeChild(node);
            }}
            el.children.push(node);
            node.parentElement = el;
            if (node.id) elements.set(node.id, node);
            return node;
        }},
        removeChild(node) {{
            const idx = el.children.indexOf(node);
            if (idx >= 0) el.children.splice(idx, 1);
            if (node.parentElement === el) {{
                node.parentElement = null;
            }}
            return node;
        }},
        querySelector(selector) {{
            if (selector.startsWith('.')) {{
                const cls = selector.slice(1);
                return el.children.find(child => child.classList && child.classList.contains(cls)) || null;
            }}
            if (selector.startsWith('#')) {{
                const id = selector.slice(1);
                return elements.get(id) || null;
            }}
            return null;
        }},
        querySelectorAll(selector) {{
            if (selector.startsWith('.')) {{
                const cls = selector.slice(1);
                return el.children.filter(child => child.classList && child.classList.contains(cls));
            }}
            return [];
        }},
        textContent: '',
        innerHTML: '',
    }};
    if (id) elements.set(id, el);
    return el;
}}

const servicesLogs = makeElement('servicesLogs');
servicesLogs.children = servicesLogs.children || [];
elements.set('servicesLogs', servicesLogs);

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
    getElementById: (id) => elements.get(id) || null,
    createElement: (tag) => makeElement(tag + '-' + Math.random().toString(16).slice(2)),
    addEventListener: (event, cb) => {{ if (typeof cb === 'function') cb(); }},
}};

global.WebSocket = class {{
    constructor() {{ this.readyState = 1; }}
    send() {{}}
    close() {{}}
}};

const {{ PilotController }} = require(joystickPath);
const controller = new PilotController({{ deferInit: true }});
controller.servicesLogs = servicesLogs;
controller.services = controller.services || {{}};
controller.serviceDetails = controller.serviceDetails || {{}};
controller.modules = controller.modules || {{}};
controller.moduleUnitMap = controller.moduleUnitMap || {{}};

const detailCalls = [];
controller.requestSystemdDetail = (unit, lines) => {{ detailCalls.push([unit, lines]); }};
const watchCalls = [];
controller.watchSystemdUnit = (unit, lines) => {{ watchCalls.push([unit, lines]); }};
controller.updateServiceLogControls = () => {{}};
controller.renderServiceDetail = () => {{}};

controller.renderServiceLogBlock = (unit) => {{
    const block = makeElement(controller.serviceId(unit) + '-detail', ['service-log-block']);
    block.parentElement = null;
    block.querySelector = () => null;
    return block;
}};

controller.renderModuleControls = (containerId, controls) => {{
    const container = elements.get(containerId) || makeElement(containerId, ['module-controls']);
    container._lastControls = JSON.stringify(controls || []);
    elements.set(containerId, container);
    return container;
}};

controller.renderModuleSection = (moduleName, moduleConfig) => {{
    const moduleId = 'module-' + moduleName.replace(/[^a-zA-Z0-9_-]/g, '-');
    const section = makeElement(moduleId, ['module-section']);
    const controls = makeElement(moduleId + '-controls', ['module-controls']);
    controls.dataset.module = moduleName;
    const systemd = makeElement(moduleId + '-systemd', ['module-systemd']);
    const title = {{ textContent: moduleConfig.name || moduleName }};
    const desc = {{ textContent: moduleConfig.description || '' }};
    section.querySelector = (selector) => {{
        if (selector === '.module-title') return title;
        if (selector === '.module-description') return desc;
        if (selector === '.module-controls') return controls;
        if (selector === '.module-systemd') return systemd;
        return null;
    }};
    section.appendChild(controls);
    section.appendChild(systemd);
    controller.servicesLogs.appendChild(section);
    elements.set(moduleId + '-controls', controls);
    elements.set(moduleId + '-systemd', systemd);
    return section;
}};

controller.updateServicesList([{{
    name: 'psyched-nav.service',
    active: 'inactive',
    enabled: 'disabled',
    status: '',
    journal: '',
}}]);

if (servicesLogs.children.length !== 1) {{ throw new Error('Service block not appended to fallback'); }}
if (detailCalls.length !== 1) {{ throw new Error('Expected initial detail request'); }}
if (watchCalls.length !== 1) {{ throw new Error('Expected initial watch registration'); }}

const modulesPayload = {{
    nav: {{
        name: 'Navigation & 3D SLAM',
        description: 'Nav module',
        controls: [],
        systemd_units: ['psyched-nav.service'],
        slug: 'nav',
    }},
}};

controller.updateModules(modulesPayload);

const moduleSystemd = elements.get('module-nav-systemd');
if (!moduleSystemd) {{ throw new Error('Module systemd container missing'); }}

const block = elements.get(controller.serviceId('psyched-nav.service') + '-detail');
if (!block) {{ throw new Error('Service block missing after modules update'); }}
if (block.parentElement !== moduleSystemd) {{ throw new Error('Service block not moved into module section'); }}
if (moduleSystemd.children.length !== 1) {{ throw new Error('Module systemd container should hold the service block'); }}
if (servicesLogs.children.length !== 1) {{ throw new Error('servicesLogs should now contain only the module section'); }}
if (detailCalls.length !== 1) {{ throw new Error('Detail should only be requested once'); }}
if (watchCalls.length !== 1) {{ throw new Error('Watch should only be requested once'); }}

if (!controller.moduleUnitMap || controller.moduleUnitMap['psyched-nav.service'] !== 'nav') {{
    throw new Error('Module unit map not populated for nav');
}}
"""

    result = subprocess.run(['node', '-e', script], capture_output=True, text=True)
    assert result.returncode == 0, f"stdout: {result.stdout}\nstderr: {result.stderr}"
