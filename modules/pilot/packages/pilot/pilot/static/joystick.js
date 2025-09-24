/**
 * Pilot Joystick Control Interface
 * Handles joystick interaction and WebSocket communication for robot control
 */

const pilotGlobals = typeof window !== 'undefined' ? window : (typeof globalThis !== 'undefined' ? globalThis : {});

const numberOrNull = (value) => (typeof value === 'number' && Number.isFinite(value) ? value : null);
const formatDecimal = (value, digits = 2) => {
    const num = numberOrNull(value);
    return num === null ? '--' : num.toFixed(digits);
};

/**
 * Build a websocket endpoint for subscribing to a ROS topic.
 *
 * Supports the following host inputs:
 *   - Bare hostnames ("api") or host:port pairs ("api:8081")
 *   - Hosts with path prefixes ("api/v1")
 *   - Protocol-qualified URLs ("https://api.example.com/base/")
 *   - Protocol-relative URLs ("//api.local:8081")
 *
 * The helper normalises schemes so HTTP ➜ ws:// and HTTPS ➜ wss:// while leaving
 * explicit ws:// / wss:// values unchanged.  Paths keep their prefix and are
 * suffixed with "/subscribe" so multiple topic sockets can share the same API
 * host without having to hardcode the full URL in the UI.
 *
 * @param {string} topic - ROS topic name (leading slash optional).
 * @param {string} rawHost - Configured API host value.
 * @param {string} defaultProtocol - Either "ws:" or "wss:" based on page scheme.
 * @returns {string|null} Fully-qualified websocket URL or null when the host is invalid.
 */
const buildTopicSubscriptionUrl = (topic, rawHost, defaultProtocol) => {
    const cleanTopic = (topic || '').trim();
    if (!cleanTopic) {
        return null;
    }
    const proto = defaultProtocol === 'wss:' ? 'wss:' : 'ws:';
    const trimmedHost = (rawHost || '').trim();
    if (!trimmedHost) {
        return null;
    }
    const collapsed = trimmedHost.replace(/\s+/g, '');
    const lower = collapsed.toLowerCase();
    let url;
    try {
        if (lower.startsWith('ws://') || lower.startsWith('wss://')) {
            url = new URL(collapsed);
        } else if (lower.startsWith('http://') || lower.startsWith('https://')) {
            url = new URL(collapsed);
            url.protocol = lower.startsWith('https://') ? 'wss:' : 'ws:';
        } else if (lower.startsWith('//')) {
            url = new URL(`${proto}${collapsed}`);
        } else {
            url = new URL(`${proto}//${collapsed}`);
        }
    } catch (error) {
        try {
            url = new URL(`${proto}//${collapsed}`);
        } catch (fallbackError) {
            console.error('subscribeToTopic: unable to resolve API host', collapsed, fallbackError);
            return null;
        }
    }
    const path = url.pathname ? url.pathname.replace(/\/+$/, '') : '';
    const suffix = `${path}/subscribe`.replace(/\/+/g, '/');
    url.pathname = suffix.startsWith('/') ? suffix : `/${suffix}`;
    url.search = `?topic=${encodeURIComponent(cleanTopic)}`;
    url.hash = '';
    return url.toString();
};

const createPilotStore = () => {
    const store = {
        connection: { status: 'disconnected', label: 'Offline', indicator: '🔴' },
        addresses: {
            web: '',
            ws: '—',
            cmdVel: '/cmd_vel',
            cmdVelSubscribers: null,
            voice: '/voice',
            voiceSubscribers: null,
        },
        twist: { linearX: '0.00', linearY: '0.00', angularZ: '0.00' },
        robot: { mode: '--', speed: '--', bumper: '--', cliff: '--', ir: '--', diag: '--' },
        battery: { percent: '--', voltage: '--', current: '--', temperature: '--', state: '--' },
        host: { cpu: '--', temp: '--', mem: '--' },
        audio: { speakingMs: '0', vadMs: '0', mic: '--' },
        gps: { fix: '--', lat: '--', lon: '--', alt: '--' },
        conversation: [],

        setConnection(status, label) {
            const normalized = (status || '').toString().toLowerCase();
            this.connection.status = normalized || 'unknown';
            this.connection.label = label || '--';
            this.connection.indicator = normalized === 'connected'
                ? '🟢'
                : normalized === 'connecting'
                    ? '🟡'
                    : '🔴';
        },

        setWebAddress(url) {
            this.addresses.web = url || '';
        },

        setWsAddress(url) {
            this.addresses.ws = url && url.trim() ? url : '—';
        },

        setCmdVelTopic(topic, subscribers) {
            this.addresses.cmdVel = topic || '--';
            this.addresses.cmdVelSubscribers = (typeof subscribers === 'number' && Number.isFinite(subscribers))
                ? subscribers
                : null;
        },

        setVoiceTopic(topic, subscribers) {
            this.addresses.voice = topic || '--';
            this.addresses.voiceSubscribers = (typeof subscribers === 'number' && Number.isFinite(subscribers))
                ? subscribers
                : null;
        },

        setTwist(twist) {
            const linear = twist && twist.linear ? twist.linear : {};
            const angular = twist && twist.angular ? twist.angular : {};
            Object.assign(this.twist, {
                linearX: formatDecimal(linear.x, 2),
                linearY: formatDecimal(linear.y, 2),
                angularZ: formatDecimal(angular.z, 2),
            });
            return this.twist;
        },

        setRobotStatus(status) {
            const modeValue = typeof status?.mode === 'string' ? status.mode : status?.mode;
            const formatMode = (code) => {
                switch (code) {
                    case 0: return 'OFF';
                    case 1: return 'PASSIVE';
                    case 2: return 'SAFE';
                    case 3: return 'FULL';
                    default: return '--';
                }
            };
            const diagCounts = Array.isArray(status?.diag_counts) ? status.diag_counts : [];
            const diagLevel = typeof status?.diag_level === 'number' ? status.diag_level : null;
            const diagText = diagCounts.length
                ? `L${diagLevel ?? 0} ok:${diagCounts[0] || 0} warn:${diagCounts[1] || 0} err:${diagCounts[2] || 0}`
                : '--';
            Object.assign(this.robot, {
                mode: typeof modeValue === 'string' ? modeValue : formatMode(modeValue),
                speed: numberOrNull(status?.speed) !== null ? numberOrNull(status.speed).toFixed(2) : '--',
                bumper: typeof status?.bumper === 'boolean' ? (status.bumper ? 'PRESSED' : 'OK') : '--',
                cliff: typeof status?.cliff === 'boolean' ? (status.cliff ? 'DETECTED' : 'OK') : '--',
                ir: numberOrNull(status?.ir_omni) !== null ? String(numberOrNull(status.ir_omni)) : '--',
                diag: diagText,
            });
            return this.robot;
        },

        setBattery(battery) {
            const clamp = (value, lo, hi) => Math.max(lo, Math.min(hi, value));
            const percent = numberOrNull(battery?.percent);
            const pctText = percent !== null ? `${clamp(percent, 0, 100).toFixed(0)}%` : '--';
            const tempC = numberOrNull(battery?.temperature);
            const tempText = tempC !== null
                ? `${tempC.toFixed(1)}°C / ${(tempC * 9 / 5 + 32).toFixed(1)}°F`
                : '--';
            const formatState = (code) => {
                switch (code) {
                    case 0: return 'Not charging';
                    case 1: return 'Reconditioning';
                    case 2: return 'Full';
                    case 3: return 'Trickle';
                    case 4: return 'Waiting';
                    case 5: return 'Fault';
                    default: return '--';
                }
            };
            Object.assign(this.battery, {
                percent: pctText,
                voltage: numberOrNull(battery?.voltage) !== null ? numberOrNull(battery.voltage).toFixed(2) : '--',
                current: numberOrNull(battery?.current) !== null ? numberOrNull(battery.current).toFixed(2) : '--',
                temperature: tempText,
                state: formatState(battery?.charging_state),
            });
            return this.battery;
        },

        setHostHealth(health) {
            Object.assign(this.host, {
                cpu: numberOrNull(health?.cpu_percent) !== null ? `${numberOrNull(health.cpu_percent).toFixed(0)}%` : '--',
                temp: numberOrNull(health?.temp_c) !== null
                    ? `${numberOrNull(health.temp_c).toFixed(0)}°C / ${(numberOrNull(health.temp_c) * 9 / 5 + 32).toFixed(0)}°F`
                    : '--',
                mem: numberOrNull(health?.mem_used_percent) !== null
                    ? `${numberOrNull(health.mem_used_percent).toFixed(0)}%`
                    : '--',
            });
            return this.host;
        },

        setAudio(audio) {
            Object.assign(this.audio, {
                speakingMs: numberOrNull(audio?.autophony_ms) !== null ? String(numberOrNull(audio.autophony_ms)) : '0',
                vadMs: numberOrNull(audio?.speech_ms) !== null ? String(numberOrNull(audio.speech_ms)) : '0',
                mic: audio?.mic
                    ? this.formatMicInfo(audio.mic)
                    : this.audio.mic,
            });
            return this.audio;
        },

        formatMicInfo(info) {
            const sr = numberOrNull(info?.sample_rate) !== null ? `${numberOrNull(info.sample_rate)} Hz` : '--';
            const ch = numberOrNull(info?.channels) !== null ? `${numberOrNull(info.channels)} ch` : '--';
            const text = `${sr}, ${ch}`;
            this.audio.mic = text;
            return text;
        },

        setGps(gps) {
            Object.assign(this.gps, {
                fix: this.formatGpsStatus(gps?.status),
                lat: numberOrNull(gps?.lat) !== null ? numberOrNull(gps.lat).toFixed(6) : '--',
                lon: numberOrNull(gps?.lon) !== null ? numberOrNull(gps.lon).toFixed(6) : '--',
                alt: numberOrNull(gps?.alt) !== null ? numberOrNull(gps.alt).toFixed(1) : '--',
            });
            return this.gps;
        },

        formatGpsStatus(code) {
            switch (code) {
                case -1:
                case 0:
                    return 'No Fix';
                case 1:
                    return 'Fix (2D/3D)';
                case 2:
                    return 'DGPS';
                default:
                    return '--';
            }
        },

        pushConversation(entry) {
            if (!entry) {
                return;
            }
            this.conversation.push(entry);
            if (this.conversation.length > 200) {
                this.conversation.shift();
            }
            return entry;
        },
    };

    return store;
};

const pilotStore = pilotGlobals.__pilotStore || createPilotStore();
if (!pilotGlobals.__pilotStore) {
    pilotGlobals.__pilotStore = pilotStore;
}
pilotGlobals.pilotStore = pilotStore;

const registerAlpineStore = () => {
    if (!pilotGlobals.Alpine || typeof pilotGlobals.Alpine.store !== 'function') {
        return;
    }
    pilotGlobals.Alpine.store('pilot', pilotStore);
    if (typeof pilotGlobals.Alpine.data === 'function') {
        pilotGlobals.Alpine.data('pilotApp', () => ({
            state: pilotStore,
            init() { },
        }));
    }
    pilotGlobals.__pilotStoreRegistered = true;
};

if (pilotGlobals.addEventListener) {
    pilotGlobals.addEventListener('alpine:init', registerAlpineStore);
}
if (pilotGlobals.Alpine && !pilotGlobals.__pilotStoreRegistered) {
    registerAlpineStore();
}

class PilotController {
    constructor(options = {}) {
        const opts = options || {};
        this.store = pilotStore;
        this.topicSubscriptions = new Map();
        this.cmdVelSubscription = null;
        this.currentCmdVelTopic = null;
        this.apiHost = this.resolveApiHost();

        this.websocket = null;
        this.isConnected = false;
        this.joystick = null;
        this.joystickKnob = null;
        this.throttleSlider = null;
        this.isSliderDragging = false;
        this.throttleDecayId = null;
        this.lastDecayTs = null;
        this.sliderLoopId = null;
        this.isDragging = false;
        this.joystickCenter = { x: 0, y: 0 };
        this.joystickRadius = 0;
        this.maxKnobDistance = 0;

        // Current velocity values
        this.currentVelocity = {
            linear: { x: 0, y: 0, z: 0 },
            angular: { x: 0, y: 0, z: 0 }
        };

        // Rate limiting
        this.lastSendTime = 0;
        this.sendRateMs = 50;

        // ✅ Added initializations
        this.services = {};
        this.serviceDetails = {};
        this.modules = {};
        this.moduleUnitMap = {};
        this.watchedUnits = new Set();

        if (!opts.deferInit) {
            this.init();
        }
    }

    init() {
        this.setupWebSocket();
        this.setupJoystick();
        this.setupButtons();
        this.setupVoice();
        // Throttle slider removed; D-Pad removed from UI — no setup needed
        this.updateAddressDisplay();

        // Battery DOM refs
        this.batteryEls = {
            fill: document.getElementById('batteryFill'),
            percent: document.getElementById('batteryPercent'),
            state: document.getElementById('batteryState'),
            percentInfo: document.getElementById('batteryPercentInfo'),
            voltage: document.getElementById('batteryVoltage'),
            current: document.getElementById('batteryCurrent'),
            temp: document.getElementById('batteryTemp'),
            stateInfo: document.getElementById('batteryStateInfo'),
        };
        this.robotEls = {
            mode: document.getElementById('robotMode'),
            speed: document.getElementById('robotSpeed'),
            bumper: document.getElementById('robotBumper'),
            cliff: document.getElementById('robotCliff'),
            ir: document.getElementById('robotIrOmni'),
            diag: document.getElementById('robotDiag'),
        };
        this.hostEls = {
            cpu: document.getElementById('cpuChip'),
            temp: document.getElementById('tempChip'),
            mem: document.getElementById('memChip'),
        };
        this.audioEls = {
            speakingMs: document.getElementById('voiceSpeakingMs'),
            vadMs: document.getElementById('vadSpeechMs'),
            micInfo: document.getElementById('micInfo'),
        };
        this.gpsEls = {
            fix: document.getElementById('gpsFixStatus'),
            lat: document.getElementById('gpsLat'),
            lon: document.getElementById('gpsLon'),
            alt: document.getElementById('gpsAlt'),
        };

        // Host health toggle setup
        this.setupHostHealthToggle();

        // Depth overlay controls
        this.setupDepthOverlayControls();

        // Send periodic keep-alive
        setInterval(() => this.sendPing(), 5000);

        // IMU DOM refs
        this.imuEls = {
            overlay: document.getElementById('imuOverlay'),
            robotYaw: document.getElementById('imuRobotYaw'),
            accelVec: document.getElementById('imuAccelVec'),
            gyroZ: document.getElementById('imuGyroZ'),
        };
        this.lastImuTs = 0;

        // Services UI
        this.servicesContainer = document.getElementById('servicesPills');
        this.servicesLogs = document.getElementById('servicesLogs');

        // Conversation UI
        this.convLog = document.getElementById('conversationLog');
        // Map canvas
        this.mapCanvas = document.getElementById('mapCanvas');
        if (this.mapCanvas) {
            this.mapCtx = this.mapCanvas.getContext('2d');
        }

        // Best-effort unwatch on page unload (backend also cleans on disconnect)
        window.addEventListener('beforeunload', () => {
            try {
                if (this.websocket && this.websocket.readyState === WebSocket.OPEN) {
                    this.watchedUnits.forEach(unit => {
                        try { this.websocket.send(JSON.stringify({ type: 'systemd', action: 'unwatch', unit })); } catch (_) { }
                    });
                }
                Array.from(this.topicSubscriptions.values()).forEach(handle => {
                    try { handle.close(); } catch (_) { }
                });
            } catch (_) { }
        });
    }

    resolveApiHost() {
        if (typeof window === 'undefined') {
            return this.apiHost || 'api';
        }
        if (typeof window.PILOT_API_HOST === 'string' && window.PILOT_API_HOST.trim()) {
            return window.PILOT_API_HOST.trim();
        }
        const attrSource = (typeof document !== 'undefined' && document) ? (document.body || document.documentElement) : null;
        if (attrSource && typeof attrSource.getAttribute === 'function') {
            const attr = attrSource.getAttribute('data-pilot-api-host');
            if (attr && attr.trim()) {
                return attr.trim();
            }
        }
        return this.apiHost || 'api';
    }

    getApiHost() {
        this.apiHost = this.resolveApiHost();
        return this.apiHost;
    }

    setupHostHealthToggle() {
        const panel = document.getElementById('hostHealthPanel');
        const toggle = document.getElementById('hostHealthToggle');
        if (!panel || !toggle) return;
        const small = window.matchMedia('(max-width: 640px)');
        const applyDefault = () => {
            const collapsed = small.matches; // default collapsed on small screens
            panel.setAttribute('data-collapsed', collapsed ? 'true' : 'false');
            toggle.setAttribute('aria-expanded', String(!collapsed));
            toggle.textContent = collapsed ? 'System ▸' : 'System ▾';
        };
        applyDefault();
        // Update on viewport changes
        try {
            small.addEventListener('change', applyDefault);
        } catch (e) {
            // Safari fallback
            small.addListener(applyDefault);
        }
        toggle.addEventListener('click', () => {
            const currently = panel.getAttribute('data-collapsed') === 'true';
            const collapsed = !currently;
            panel.setAttribute('data-collapsed', collapsed ? 'true' : 'false');
            toggle.setAttribute('aria-expanded', String(!collapsed));
            toggle.textContent = collapsed ? 'System ▸' : 'System ▾';
        });
    }

    setupDepthOverlayControls() {
        const toggle = document.getElementById('depthOverlayToggle');
        const slider = document.getElementById('depthOpacity');
        const depthEl = document.getElementById('depth-image');
        if (!depthEl) return;
        if (toggle) {
            toggle.addEventListener('change', () => {
                depthEl.setAttribute('data-visible', toggle.checked ? 'true' : 'false');
            });
        }
        if (slider) {
            const setOpacity = (v) => {
                const op = Math.max(0, Math.min(100, parseInt(v, 10))) / 100.0;
                depthEl.setAttribute('data-opacity', String(op));
            };
            // set initial
            setOpacity(slider.value);
            slider.addEventListener('input', () => setOpacity(slider.value));
        }
    }

    setupWebSocket() {
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const host = window.location.hostname || 'localhost';
        const basePort = window.location.port ? parseInt(window.location.port) : (window.location.protocol === 'https:' ? 443 : 80);
        const candidates = [];
        // Primary: port + 1 per backend default
        candidates.push(`${protocol}//${host}:${basePort + 1}`);
        // Fallbacks: known defaults 8081, 9091
        candidates.push(`${protocol}//${host}:8081`);
        candidates.push(`${protocol}//${host}:9091`);

        const tryConnect = (urls, idx = 0) => {
            if (idx >= urls.length) {
                this.updateStatus('Disconnected', 'disconnected');
                return;
            }
            const wsUrl = urls[idx];
            try {
                this.websocket = new WebSocket(wsUrl);

                this.websocket.onopen = () => {
                    this.isConnected = true;
                    this.updateStatus('Connected', 'connected');
                    console.log('WebSocket connected', wsUrl);
                    this.updateWsAddress(wsUrl);
                    // Send immediate ping to fetch status
                    this.sendPing();
                    // Ask for services list
                    this.requestSystemdList();
                };

                this.websocket.onclose = () => {
                    const wasConnected = this.isConnected;
                    this.isConnected = false;
                    this.updateStatus('Disconnected', 'disconnected');
                    this.updateWsAddress('—');
                    console.log('WebSocket disconnected');
                    // If we never established, try next candidate; else schedule reconnect to same
                    if (!wasConnected) {
                        tryConnect(urls, idx + 1);
                    } else {
                        setTimeout(() => this.setupWebSocket(), 3000);
                    }
                };

                this.websocket.onerror = (error) => {
                    console.error('WebSocket error:', error);
                    this.updateStatus('Connection Error', 'disconnected');
                };

                this.websocket.onmessage = (event) => {
                    this.handleWebSocketMessage(event.data);
                };
            } catch (error) {
                console.error('Failed to create WebSocket:', error);
                this.updateStatus('Connection Failed', 'disconnected');
                tryConnect(urls, idx + 1);
            }
        };

        tryConnect(candidates);
    }

    setupJoystick() {
        this.joystick = document.getElementById('joystick');
        this.joystickKnob = document.getElementById('joystickKnob');

        const rect = this.joystick.getBoundingClientRect();
        this.joystickRadius = rect.width / 2;
        this.maxKnobDistance = this.joystickRadius - 30; // Account for knob size

        // Mouse events
        // Allow starting drag from knob or anywhere inside joystick circle
        this.joystick.addEventListener('mousedown', this.onJoystickStart.bind(this));
        document.addEventListener('mousemove', this.onJoystickMove.bind(this));
        document.addEventListener('mouseup', this.onJoystickEnd.bind(this));

        // Touch events
        this.joystick.addEventListener('touchstart', this.onJoystickStart.bind(this), { passive: false });
        document.addEventListener('touchmove', this.onJoystickMove.bind(this), { passive: false });
        document.addEventListener('touchend', this.onJoystickEnd.bind(this), { passive: false });
        document.addEventListener('touchcancel', this.onJoystickEnd.bind(this), { passive: false });

        // Prevent context menu
        this.joystick.addEventListener('contextmenu', e => e.preventDefault());

        // Update joystick center on window resize
        window.addEventListener('resize', () => {
            const rect = this.joystick.getBoundingClientRect();
            this.joystickRadius = rect.width / 2;
            this.maxKnobDistance = this.joystickRadius - 30;
        });
    }

    setupButtons() {
        const stopButton = document.getElementById('stopButton');
        const resetButton = document.getElementById('resetButton');
        const mapSave = document.getElementById('mapSaveBtn');
        const mapRecord = document.getElementById('mapRecordBtn');

        stopButton.addEventListener('click', () => {
            this.emergencyStop();
        });

        resetButton.addEventListener('click', () => {
            this.resetJoystick();
        });

        if (mapSave) {
            mapSave.addEventListener('click', () => {
                if (!this.websocket || !this.isConnected) return;
                try {
                    // Ask backend to save map; optional name can be added later
                    this.websocket.send(JSON.stringify({ type: 'save_map', name: 'nav_map' }));
                } catch (e) { console.error('Failed to request save_map', e); }
            });
        }

        if (mapRecord) {
            mapRecord.addEventListener('click', () => {
                if (!this.websocket || !this.isConnected) return;
                // Toggle recording state locally by disabling button and sending start/stop
                if (!mapRecord.dataset.recording || mapRecord.dataset.recording === 'false') {
                    try {
                        this.websocket.send(JSON.stringify({ type: 'record_bag', name: 'nav_record' }));
                        mapRecord.dataset.recording = 'starting';
                        mapRecord.textContent = 'Recording...';
                        mapRecord.disabled = true;
                        // We'll enable and mark true on ack
                    } catch (e) { console.error('Failed to start recording', e); }
                } else {
                    try {
                        this.websocket.send(JSON.stringify({ type: 'record_bag_stop' }));
                        mapRecord.dataset.recording = 'stopping';
                        mapRecord.disabled = true;
                        mapRecord.textContent = 'Stopping...';
                    } catch (e) { console.error('Failed to stop recording', e); }
                }
            });
        }
    }

    setupVoice() {
        const input = document.getElementById('voiceInput');
        const button = document.getElementById('voiceSendButton');
        const marco = document.getElementById('marcoButton');
        const pauseBtn = document.getElementById('voicePause');
        const resumeBtn = document.getElementById('voiceResume');
        const clearBtn = document.getElementById('voiceClear');
        const volSlider = document.getElementById('voiceVolume');
        const volValue = document.getElementById('voiceVolumeValue');
        if (!input || !button) return;

        const send = () => {
            const text = (input.value || '').trim();
            if (!text) return;
            if (!this.isConnected || !this.websocket) return;
            try {
                this.websocket.send(JSON.stringify({ type: 'voice', text }));
                // optimistic clear
                input.value = '';
            } catch (e) {
                console.error('Failed to send voice message:', e);
            }
        };

        button.addEventListener('click', send);
        if (marco) {
            marco.addEventListener('click', () => {
                if (!this.isConnected || !this.websocket) return;
                try {
                    this.websocket.send(JSON.stringify({ type: 'voice', text: 'Polo!' }));
                } catch (e) {
                    console.error('Failed to send marco message:', e);
                }
            });
        }
        input.addEventListener('keydown', (e) => {
            if (e.key === 'Enter') {
                e.preventDefault();
                send();
            }
        });

        const sendControl = (action) => {
            if (!this.isConnected || !this.websocket) return;
            try {
                this.websocket.send(JSON.stringify({ type: 'voice_control', action }));
            } catch (e) {
                console.error('Failed to send voice control:', e);
            }
        };
        if (pauseBtn) pauseBtn.addEventListener('click', () => sendControl('pause'));
        if (resumeBtn) resumeBtn.addEventListener('click', () => sendControl('resume'));
        if (clearBtn) clearBtn.addEventListener('click', () => sendControl('clear'));

        const sendVolume = (value) => {
            if (!this.isConnected || !this.websocket) return;
            try {
                this.websocket.send(JSON.stringify({ type: 'voice_volume', value }));
            } catch (e) {
                console.error('Failed to send voice volume:', e);
            }
        };
        if (volSlider) {
            // Slider 0..200 maps to 0.0..2.0 (espeak supports >1x, clamp backend to 2.0)
            const updateLabel = () => { if (volValue) volValue.textContent = `${volSlider.value}%`; };
            updateLabel();
            volSlider.addEventListener('input', () => {
                updateLabel();
                const frac = Math.max(0, Math.min(200, parseInt(volSlider.value, 10))) / 100.0;
                // Debounce a bit by sending at most every 150ms
                clearTimeout(this._volTimer);
                this._volTimer = setTimeout(() => sendVolume(frac), 150);
            });
        }
    }

    updateAddressDisplay() {
        const webAddress = `${window.location.protocol}//${window.location.host}`;
        // Initial guess for WS; will be updated on successful connect
        const wsPort = parseInt(window.location.port) + 1;
        const wsProtocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const wsAddress = `${wsProtocol}//${window.location.hostname}:${wsPort}`;
        if (this.store && typeof this.store.setWebAddress === 'function') {
            this.store.setWebAddress(webAddress);
        }
        if (this.store && typeof this.store.setWsAddress === 'function') {
            this.store.setWsAddress(wsAddress);
        }
        document.getElementById('webAddress').textContent = webAddress;
        document.getElementById('wsAddress').textContent = wsAddress;
    }

    updateWsAddress(url) {
        const el = document.getElementById('wsAddress');
        const value = url && url.trim() ? url : '—';
        if (this.store && typeof this.store.setWsAddress === 'function') {
            this.store.setWsAddress(value);
        }
        if (el) el.textContent = value;
    }

    subscribeToTopic(topic, options = {}) {
        const cleanTopic = (topic || '').trim();
        if (!cleanTopic) {
            return null;
        }
        const key = (options.key || cleanTopic) || cleanTopic;
        const protocol = (typeof window !== 'undefined' && window.location && window.location.protocol === 'https:') ? 'wss:' : 'ws:';
        const host = options.apiHost || this.getApiHost() || 'api';
        const endpoint = buildTopicSubscriptionUrl(cleanTopic, host, protocol);
        if (!endpoint) {
            console.error('Failed to build topic subscription URL for', cleanTopic, host);
            return null;
        }
        try {
            if (this.topicSubscriptions.has(key)) {
                const existing = this.topicSubscriptions.get(key);
                if (existing && typeof existing.close === 'function') {
                    existing.close();
                }
                this.topicSubscriptions.delete(key);
            }
            const socket = new WebSocket(endpoint);
            const handle = {
                topic: cleanTopic,
                socket,
                close: () => {
                    try { socket.close(); } catch (_) { }
                    this.topicSubscriptions.delete(key);
                },
            };
            const parseJson = options.parseJson !== false;
            socket.addEventListener('message', (event) => {
                let payload = event.data;
                if (parseJson && typeof payload === 'string') {
                    try { payload = JSON.parse(payload); } catch (_) { /* ignore malformed payload */ }
                }
                if (typeof options.onMessage === 'function') {
                    try { options.onMessage(payload); } catch (error) { console.error('subscribeToTopic onMessage error:', error); }
                }
                if (typeof options.onTopic === 'function' && payload && typeof payload === 'object' && payload.type === 'topic') {
                    try { options.onTopic(payload.message, payload); } catch (error) { console.error('subscribeToTopic onTopic error:', error); }
                }
            });
            socket.addEventListener('close', () => {
                this.topicSubscriptions.delete(key);
                if (typeof options.onClose === 'function') {
                    try { options.onClose(); } catch (_) { }
                }
            });
            socket.addEventListener('error', (err) => {
                if (typeof options.onError === 'function') {
                    try { options.onError(err); } catch (_) { }
                }
            });
            this.topicSubscriptions.set(key, handle);
            return handle;
        } catch (error) {
            console.error('Failed to subscribe to topic:', error);
            return null;
        }
    }

    ensureCmdVelSubscription(topic) {
        const cleanTopic = (topic || '').trim();
        if (!cleanTopic) {
            return;
        }
        if (this.cmdVelSubscription && this.cmdVelSubscription.topic === cleanTopic) {
            return;
        }
        if (this.cmdVelSubscription && typeof this.cmdVelSubscription.close === 'function') {
            try { this.cmdVelSubscription.close(); } catch (_) { }
        }
        const handle = this.subscribeToTopic(cleanTopic, {
            key: 'cmd_vel',
            onTopic: (message) => this.handleCmdVelTopicMessage(message),
        });
        if (handle) {
            this.cmdVelSubscription = handle;
        }
    }

    handleCmdVelTopicMessage(message) {
        if (!message) {
            return;
        }
        const linearSrc = message.linear || {};
        const angularSrc = message.angular || {};
        const twist = {
            linear: {
                x: numberOrNull(linearSrc.x ?? message.linear_x ?? message.x) ?? 0,
                y: numberOrNull(linearSrc.y ?? message.linear_y ?? message.y) ?? 0,
                z: numberOrNull(linearSrc.z ?? message.linear_z ?? 0) ?? 0,
            },
            angular: {
                x: numberOrNull(angularSrc.x ?? message.angular_x ?? 0) ?? 0,
                y: numberOrNull(angularSrc.y ?? message.angular_y ?? 0) ?? 0,
                z: numberOrNull(angularSrc.z ?? message.angular_z ?? message.z) ?? 0,
            },
        };
        this.currentVelocity = twist;
        this.renderTwist(twist);
    }

    onJoystickStart(event) {
        this.isDragging = true;
        this.joystick.setAttribute('data-dragging', 'true');

        // Update joystick center position
        const rect = this.joystick.getBoundingClientRect();
        this.joystickCenter = {
            x: rect.left + rect.width / 2,
            y: rect.top + rect.height / 2
        };

        // Immediately position knob to the touch/click point if it's within the joystick circle
        const clientX = event.touches ? event.touches[0].clientX : event.clientX;
        const clientY = event.touches ? event.touches[0].clientY : event.clientY;

        const deltaX = clientX - this.joystickCenter.x;
        const deltaY = clientY - this.joystickCenter.y;
        const distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        if (distance <= this.joystickRadius) {
            // Clamp within maxKnobDistance
            const limitedDistance = Math.min(distance, this.maxKnobDistance);
            const angle = Math.atan2(deltaY, deltaX);
            const knobX = limitedDistance * Math.cos(angle);
            const knobY = limitedDistance * Math.sin(angle);
            this.joystickKnob.style.transform = `translate(${knobX - 30}px, ${knobY - 30}px)`;

            // Update velocities once on start
            this.updateVelocitiesFromInput({ clientX, clientY });
        }

        event.preventDefault();
    }

    onJoystickMove(event) {
        if (!this.isDragging) return;

        this.updateVelocitiesFromInput(event);

        event.preventDefault();
    }

    onJoystickEnd(event) {
        if (!this.isDragging) return;

        this.isDragging = false;
        this.joystick.removeAttribute('data-dragging');

        // Return knob to center with smooth animation
        this.joystickKnob.style.transform = 'translate(-50%, -50%)';

        // Stop turn, keep throttle where slider is (no auto zero of linear x)
        const linearX = this.getThrottleValue();
        this.currentVelocity = {
            linear: { x: linearX, y: 0, z: 0 },
            angular: { x: 0, y: 0, z: 0 }
        };

        this.updateVelocityDisplay();
        this.sendVelocityCommand();

        event.preventDefault();
    }

    setupSlider() { /* removed */ }

    setupDpad() {
        // D-Pad removed from markup; leave as no-op to avoid errors from older builds
        return;
    }

    startDpadLoop() {
        if (this.dpadLoopId != null) return;
        const maxLinearVel = 0.7; // m/s for forward/backward via D-pad
        const maxAngularVel = 1.8; // rad/s for turning via D-pad
        const step = () => {
            const forward = this.dpadState.up ? 1 : 0;
            const backward = this.dpadState.down ? 1 : 0;
            const left = this.dpadState.left ? 1 : 0;
            const right = this.dpadState.right ? 1 : 0;

            // Compute velocities: allow combos (e.g., forward + left)
            const x = (forward - backward) * maxLinearVel;
            const z = (right - left) * maxAngularVel * -1; // negative for left positive convention

            this.currentVelocity = {
                linear: { x, y: 0, z: 0 },
                angular: { x: 0, y: 0, z }
            };
            this.updateVelocityDisplay();
            this.sendVelocityCommand();

            if (!this.dpadState.up && !this.dpadState.down && !this.dpadState.left && !this.dpadState.right) {
                this.stopDpadLoop();
                return;
            }
            this.dpadLoopId = window.requestAnimationFrame(step);
        };
        this.dpadLoopId = window.requestAnimationFrame(step);
    }

    stopDpadLoop() {
        if (this.dpadLoopId != null) {
            window.cancelAnimationFrame(this.dpadLoopId);
            this.dpadLoopId = null;
        }
    }

    getThrottleValue() { return 0; }

    startThrottleDecay() {
        // Smoothly reduce slider value toward 0 after release
        this.stopThrottleDecay();
        const epsilon = 0.005; // stop threshold in slider units [-1..1]
        const decayPerSecond = 1.5; // linear decay units per second (gentler)
        const step = (ts) => {
            if (this.isSliderDragging) { this.stopThrottleDecay(); return; }
            if (this.lastDecayTs == null) this.lastDecayTs = ts;
            const dt = Math.max(0, (ts - this.lastDecayTs) / 1000);
            this.lastDecayTs = ts;
            let v = parseFloat(this.throttleSlider.value);
            if (Number.isNaN(v)) v = 0;
            if (Math.abs(v) <= epsilon) {
                this.throttleSlider.value = '0';
                // Update velocity to exact zero
                const angularZ = this.currentVelocity.angular.z || 0;
                this.currentVelocity = {
                    linear: { x: 0, y: 0, z: 0 },
                    angular: { x: 0, y: 0, z: angularZ }
                };
                this.updateVelocityDisplay();
                this.sendVelocityCommand();
                this.stopThrottleDecay();
                return;
            }
            const delta = Math.sign(v) * decayPerSecond * dt;
            let newV = v - delta;
            // Prevent overshoot past 0
            if (Math.sign(v) !== Math.sign(newV)) newV = 0;
            this.throttleSlider.value = String(Math.max(-1, Math.min(1, newV)));
            // Reflect in velocity and send
            const angularZ = this.currentVelocity.angular.z || 0;
            this.currentVelocity = {
                linear: { x: this.getThrottleValue(), y: 0, z: 0 },
                angular: { x: 0, y: 0, z: angularZ }
            };
            this.updateVelocityDisplay();
            this.sendVelocityCommand();
            this.throttleDecayId = window.requestAnimationFrame(step);
        };
        this.lastDecayTs = null;
        this.throttleDecayId = window.requestAnimationFrame(step);
    }

    stopThrottleDecay() {
        if (this.throttleDecayId != null) {
            window.cancelAnimationFrame(this.throttleDecayId);
            this.throttleDecayId = null;
        }
        this.lastDecayTs = null;
    }

    startSliderDragLoop() {
        // Continuously reflect slider value in cmd_vel while dragging
        if (this.sliderLoopId != null) return;
        const step = () => {
            if (!this.isSliderDragging) { this.stopSliderDragLoop(); return; }
            const linearX = this.getThrottleValue();
            const angularZ = this.currentVelocity.angular.z || 0;
            this.currentVelocity = {
                linear: { x: linearX, y: 0, z: 0 },
                angular: { x: 0, y: 0, z: angularZ }
            };
            this.updateVelocityDisplay();
            this.sendVelocityCommand();
            this.sliderLoopId = window.requestAnimationFrame(step);
        };
        this.sliderLoopId = window.requestAnimationFrame(step);
    }

    stopSliderDragLoop() {
        if (this.sliderLoopId != null) {
            window.cancelAnimationFrame(this.sliderLoopId);
            this.sliderLoopId = null;
        }
    }

    updateVelocitiesFromInput(event) {
        const clientX = event.touches ? event.touches[0].clientX : event.clientX;
        const clientY = event.touches ? event.touches[0].clientY : event.clientY;

        // Calculate distance from center
        const deltaX = clientX - this.joystickCenter.x;
        const deltaY = clientY - this.joystickCenter.y;
        const distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

        // Limit to maximum distance
        const limitedDistance = Math.min(distance, this.maxKnobDistance);
        const angle = Math.atan2(deltaY, deltaX);

        const knobX = limitedDistance * Math.cos(angle);
        const knobY = limitedDistance * Math.sin(angle);

        // Update knob position
        this.joystickKnob.style.transform = `translate(${knobX - 30}px, ${knobY - 30}px)`;

        // Map joystick vector to velocities:
        // Up (negative Y in screen) => +linear.x; Left => +angular.z (conventional left turn positive)
        const nx = knobX / this.maxKnobDistance; // -1..1 (right positive)
        const ny = knobY / this.maxKnobDistance; // -1..1 (down positive)
        // Apply small deadzone to avoid jitter
        const dead = 0.06;
        const dz = (v) => (Math.abs(v) < dead ? 0 : v);
        const nxDZ = dz(nx), nyDZ = dz(ny);

        const maxLinearVel = 0.7;   // m/s
        const maxAngularVel = 2.0;  // rad/s
        const linearX = -nyDZ * maxLinearVel;      // up => forward
        const angularZ = -nxDZ * maxAngularVel;    // left => positive turn

        this.currentVelocity = {
            linear: { x: linearX, y: 0.0, z: 0.0 },
            angular: { x: 0.0, y: 0.0, z: angularZ }
        };

        this.updateVelocityDisplay();
        this.sendVelocityCommand();
    }

    renderTwist(twist) {
        const formatted = this.store && typeof this.store.setTwist === 'function'
            ? this.store.setTwist(twist)
            : {
                linearX: formatDecimal(twist?.linear?.x, 2),
                linearY: formatDecimal(twist?.linear?.y, 2),
                angularZ: formatDecimal(twist?.angular?.z, 2),
            };
        const apply = (id, value) => {
            const el = document.getElementById(id);
            if (el) el.textContent = value;
        };
        apply('linearX', formatted.linearX);
        apply('linearY', formatted.linearY);
        apply('angularZ', formatted.angularZ);
    }

    updateVelocityDisplay() {
        this.renderTwist(this.currentVelocity);
    }

    sendVelocityCommand() {
        const now = Date.now();
        if (now - this.lastSendTime < this.sendRateMs) {
            return; // Rate limiting
        }
        this.lastSendTime = now;

        if (!this.isConnected || !this.websocket) {
            return;
        }

        const message = {
            type: 'cmd_vel',
            linear: this.currentVelocity.linear,
            angular: this.currentVelocity.angular
        };

        try {
            this.websocket.send(JSON.stringify(message));
        } catch (error) {
            console.error('Failed to send velocity command:', error);
        }
    }

    sendPing() {
        if (!this.isConnected || !this.websocket) {
            return;
        }

        const message = { type: 'ping' };

        try {
            this.websocket.send(JSON.stringify(message));
        } catch (error) {
            console.error('Failed to send ping:', error);
        }
    }

    emergencyStop() {
        this.currentVelocity = {
            linear: { x: 0, y: 0, z: 0 },
            angular: { x: 0, y: 0, z: 0 }
        };

        this.updateVelocityDisplay();
        this.sendVelocityCommand();
        this.resetJoystick();

        console.log('Emergency stop activated');
    }

    resetJoystick() {
        this.isDragging = false;
        this.joystick.removeAttribute('data-dragging');
        this.joystickKnob.style.transform = 'translate(-50%, -50%)';

        // Center slider too
        if (this.throttleSlider) {
            this.throttleSlider.value = '0';
        }
        this.stopThrottleDecay();

        this.currentVelocity = {
            linear: { x: 0, y: 0, z: 0 },
            angular: { x: 0, y: 0, z: 0 }
        };

        this.updateVelocityDisplay();
        this.sendVelocityCommand();
    }

    updateStatus(text, className) {
        const label = text || '--';
        if (this.store && typeof this.store.setConnection === 'function') {
            this.store.setConnection(className, label);
        }
        const banner = document.querySelector('[data-source="connectionStatus"]');
        if (banner) {
            banner.textContent = label;
        }
        const indicator = document.getElementById('statusIndicator');
        if (indicator) {
            const indicatorText = this.store ? this.store.connection.indicator : (className === 'connected' ? '🟢' : className === 'disconnected' ? '🔴' : '🟡');
            indicator.textContent = indicatorText;
        }
    }

    handleWebSocketMessage(data) {
        try {
            const message = JSON.parse(data);

            switch (message.type) {
                case 'ack':
                    // Command acknowledged
                    console.log('Command acknowledged:', message.cmd_vel);
                    break;
                case 'pong':
                    // Ping response
                    if (message.cmd_vel_topic) {
                        this.updateCmdVelTopic(message.cmd_vel_topic, message.publisher_matched_count);
                        this.updateStatus(`Connected`, 'connected');
                    } else {
                        console.log('Ping response received');
                    }
                    if (message.voice_topic) {
                        this.updateVoiceTopic(message.voice_topic, message.voice_subscriber_count);
                    }
                    // Update ws footer address if backend included server info later (optional)
                    if (message.imu_topic) {
                        // could show somewhere if desired
                    }
                    if (message.battery) {
                        this.updateBattery(message.battery);
                    }
                    if (message.robot_status) {
                        this.updateRobotStatus(message.robot_status);
                    }
                    if (message.host_health) {
                        this.updateHostHealth(message.host_health);
                    }
                    if (message.audio) {
                        this.updateAudioStatus(message.audio);
                    }
                    if (message.gps_fix) {
                        this.updateGps(message.gps_fix);
                    }
                    if (Array.isArray(message.systemd_services)) {
                        this.updateServicesList(message.systemd_services);
                    }
                    if (message.modules) {
                        this.updateModules(message.modules);
                    }
                    break;
                case 'snapshot':
                    // Full stacked snapshot from backend: reuse existing update helpers
                    if (message.imu) this.updateImu(message.imu);
                    if (message.battery) this.updateBattery(message.battery);
                    if (message.robot_status) this.updateRobotStatus(message.robot_status);
                    if (message.host_health) this.updateHostHealth(message.host_health);
                    if (message.audio) this.updateAudioStatus(message.audio);
                    if (message.gps_fix) this.updateGps(message.gps_fix);
                    if (Array.isArray(message.systemd_services)) this.updateServicesList(message.systemd_services);
                    if (message.modules) this.updateModules(message.modules);
                    break;
                case 'record_started':
                    try {
                        const btn = document.getElementById('mapRecordBtn');
                        if (btn) {
                            btn.dataset.recording = 'true';
                            btn.disabled = false;
                            btn.textContent = 'Stop Recording';
                        }
                    } catch (e) { }
                    break;
                case 'record_stopped':
                    try {
                        const btn = document.getElementById('mapRecordBtn');
                        if (btn) {
                            btn.dataset.recording = 'false';
                            btn.disabled = false;
                            btn.textContent = 'Record Session';
                        }
                    } catch (e) { }
                    break;
                case 'save_map_ack':
                    try {
                        const ms = document.getElementById('mapSaveBtn');
                        if (ms) {
                            ms.textContent = 'Save Requested';
                            setTimeout(() => { ms.textContent = 'Save Map'; }, 2000);
                        }
                    } catch (e) { }
                    break;
                case 'status':
                    if (message.cmd_vel_topic) {
                        this.updateCmdVelTopic(message.cmd_vel_topic, message.publisher_matched_count);
                        this.updateStatus(`Connected`, 'connected');
                    }
                    if (message.voice_topic) {
                        this.updateVoiceTopic(message.voice_topic, message.voice_subscriber_count);
                    }
                    // keep footer in sync (ws address set on connect already)
                    if (message.imu_topic) {
                        // could show somewhere if desired
                    }
                    if (message.battery) {
                        this.updateBattery(message.battery);
                    }
                    if (message.robot_status) {
                        this.updateRobotStatus(message.robot_status);
                    }
                    if (message.host_health) {
                        this.updateHostHealth(message.host_health);
                    }
                    if (message.audio) {
                        this.updateAudioStatus(message.audio);
                    }
                    if (message.gps_fix) {
                        this.updateGps(message.gps_fix);
                    }
                    if (Array.isArray(message.systemd_services)) {
                        this.updateServicesList(message.systemd_services);
                    }
                    if (message.modules) {
                        this.updateModules(message.modules);
                    }
                    break;
                case 'systemd':
                    if (Array.isArray(message.services)) {
                        this.updateServicesList(message.services);
                    }
                    if (message.unit && message.status) {
                        this.updateService(message.status);
                        if (message.error) this.flashServiceError(message.unit, message.error);
                    }
                    if (message.unit && message.detail) {
                        this.renderServiceDetail(message.unit, message.detail);
                    }
                    if (!message.unit && message.error) {
                        console.warn('systemd error:', message.error);
                    }
                    break;
                case 'imu':
                    this.updateImu(message);
                    break;
                case 'battery':
                    this.updateBattery(message);
                    break;
                case 'robot_status':
                    this.updateRobotStatus(message);
                    break;
                case 'host_health':
                    this.updateHostHealth(message);
                    break;
                case 'audio_status':
                    this.updateAudioStatus(message);
                    break;
                case 'audio_info':
                    this.updateMicInfo(message);
                    break;
                case 'gps_fix':
                    this.updateGps(message);
                    break;
                case 'conversation':
                    this.addConversation(message);
                    break;
                case 'image':
                    if (message.topic === '/image_raw') {
                        const el = document.getElementById('camera-image');
                        if (el) el.src = message.data;
                    } else if (message.topic === '/depth/image_raw') {
                        const el = document.getElementById('depth-image');
                        if (el) el.src = message.data;
                    }
                    break;
                case 'map':
                    if (this.mapCtx) {
                        const img = new Image();
                        img.onload = () => {
                            const cw = this.mapCanvas.width;
                            const ch = this.mapCanvas.height;
                            const ar = img.width / img.height;
                            let dw = cw, dh = ch;
                            if (cw / ch > ar) { dw = ch * ar; } else { dh = cw / ar; }
                            this.mapCtx.clearRect(0, 0, cw, ch);
                            this.mapCtx.drawImage(img, (cw - dw) / 2, (ch - dh) / 2, dw, dh);
                        };
                        img.src = message.data;
                    }
                    break;
                case 'map_raw':
                    if (this.mapCtx && message.width && message.height && message.data) {
                        const w = parseInt(message.width, 10);
                        const h = parseInt(message.height, 10);
                        const raw = atob(message.data);
                        const arr = new Uint8Array(raw.length);
                        for (let i = 0; i < raw.length; i++) arr[i] = raw.charCodeAt(i);

                        const canvas = this.mapCanvas;
                        const maxSize = 800;
                        const scale = w > h ? Math.min(maxSize / w, 1.0) : Math.min(maxSize / h, 1.0);
                        canvas.width = Math.max(200, Math.floor(w * scale));
                        canvas.height = Math.max(200, Math.floor(h * scale));

                        const imgData = this.mapCtx.createImageData(w, h);
                        for (let y = 0; y < h; y++) {
                            for (let x = 0; x < w; x++) {
                                const idx = y * w + x;
                                let v = (arr[idx] & 0xFF) - 128;
                                let color = 127;
                                if (v === -1) color = 127;
                                else if (v === 0) color = 255;
                                else if (v > 0) color = 0;
                                const p = ((h - 1 - y) * w + x) * 4;
                                imgData.data[p] = color;
                                imgData.data[p + 1] = color;
                                imgData.data[p + 2] = color;
                                imgData.data[p + 3] = 255;
                            }
                        }
                        const off = document.createElement('canvas');
                        off.width = w; off.height = h;
                        const offCtx = off.getContext('2d');
                        offCtx.putImageData(imgData, 0, 0);
                        this.mapCtx.clearRect(0, 0, canvas.width, canvas.height);
                        this.mapCtx.drawImage(off, 0, 0, canvas.width, canvas.height);
                    }
                    break;
                default:
                    console.log('Unknown message type:', message.type);
            }
        } catch (error) {
            console.error('Failed to parse WebSocket message:', error);
        }
    }

    renderModuleSection(moduleName, moduleConfig) {
        const moduleId = 'module-' + moduleName.replace(/[^a-zA-Z0-9_-]/g, '-');
        const section = document.createElement('div');
        // mark module section semantically
        section.setAttribute('data-role', 'module-section');
        section.id = moduleId;

        section.innerHTML = `
        <div class="module-header">
            <h3 class="module-title">${moduleConfig.name}</h3>
            <p class="module-description">${moduleConfig.description || ''}</p>
        </div>
        <div class="module-controls" id="${moduleId}-controls"></div>
        <div class="module-systemd" id="${moduleId}-systemd"></div>
    `;

        if (this.servicesLogs) {
            this.servicesLogs.appendChild(section);
        }

        // Render module controls initially
        this.renderModuleControls(moduleId + '-controls', moduleConfig.controls, moduleName);
    }


    addConversation(msg) {
        if (!this.convLog) return;
        const el = document.createElement('div');
        // semantic marker for conversation entries
        el.setAttribute('data-role', 'conversation-entry');
        const role = (msg.role || '').toString().trim() || 'assistant';
        const content = (msg.content || '').toString();
        el.innerHTML = `<span class="role">${role}:</span> <span class="content"></span>`;
        const c = el.querySelector('.content');
        if (c) c.textContent = content;
        this.convLog.appendChild(el);
        // autoscroll to bottom
        this.convLog.scrollTop = this.convLog.scrollHeight;
        if (this.store && typeof this.store.pushConversation === 'function') {
            this.store.pushConversation({ role, content });
        }
    }

    updateAudioStatus(a) {
        const els = this.audioEls;
        const formatted = this.store && typeof this.store.setAudio === 'function'
            ? this.store.setAudio(a)
            : null;
        if (!els) return;
        if (els.speakingMs) {
            const speaking = formatted ? formatted.speakingMs : (typeof a.autophony_ms === 'number' ? String(a.autophony_ms) : '0');
            els.speakingMs.textContent = speaking;
        }
        if (els.vadMs) {
            const vad = formatted ? formatted.vadMs : (typeof a.speech_ms === 'number' ? String(a.speech_ms) : '0');
            els.vadMs.textContent = vad;
        }
        if (a.mic) {
            this.updateMicInfo(a.mic);
        } else if (formatted && els.micInfo) {
            els.micInfo.textContent = formatted.mic;
        }
    }

    updateMicInfo(m) {
        const el = this.audioEls?.micInfo;
        const text = this.store && typeof this.store.formatMicInfo === 'function'
            ? this.store.formatMicInfo(m)
            : (() => {
                const sr = (typeof m?.sample_rate === 'number' && m.sample_rate) ? `${m.sample_rate} Hz` : '--';
                const ch = (typeof m?.channels === 'number' && m.channels) ? `${m.channels} ch` : '--';
                return `${sr}, ${ch}`;
            })();
        if (el) el.textContent = text;
    }

    updateGps(g) {
        const els = this.gpsEls;
        const formatted = this.store && typeof this.store.setGps === 'function'
            ? this.store.setGps(g)
            : null;
        if (!els) return;
        if (els.fix) els.fix.textContent = formatted ? formatted.fix : this.formatGpsStatus(g.status);
        if (els.lat) els.lat.textContent = formatted ? formatted.lat : (typeof g.lat === 'number' ? g.lat.toFixed(6) : '--');
        if (els.lon) els.lon.textContent = formatted ? formatted.lon : (typeof g.lon === 'number' ? g.lon.toFixed(6) : '--');
        if (els.alt) els.alt.textContent = formatted ? formatted.alt : (typeof g.alt === 'number' ? g.alt.toFixed(1) : '--');
    }

    formatGpsStatus(code) {
        switch (code) {
            case -1: return 'No Fix';
            case 0: return 'No Fix';
            case 1: return 'Fix (2D/3D)';
            case 2: return 'DGPS';
            default: return '--';
        }
    }

    // =====================
    // Systemd UI and actions
    // =====================
    requestSystemdList() {
        if (!this.websocket) return;
        try { this.websocket.send(JSON.stringify({ type: 'systemd', action: 'list' })); } catch (e) { /* noop */ }
    }

    updateServicesList(services) {
        if (!Array.isArray(services)) return;
        // Merge/update map without clearing DOM to prevent blink
        services.forEach(svc => {
            if (!svc || !svc.name) return;
            this.services[svc.name] = svc;
            const moduleHint = (svc.module && typeof svc.module === 'string') ? svc.module : null;
            if (moduleHint) {
                if (!this.moduleUnitMap) this.moduleUnitMap = {};
                if (!this.moduleUnitMap[svc.name]) {
                    this.moduleUnitMap[svc.name] = moduleHint;
                }
            }
            // Update legacy pill if still present (container may be absent)
            if (this.servicesContainer) {
                const pillId = this.serviceId(svc.name);
                let pill = document.getElementById(pillId);
                if (!pill) {
                    pill = this.renderServicePill(svc);
                    this.servicesContainer.appendChild(pill);
                } else {
                    this.populateServicePill(pill, svc);
                }
            }

            // Update module-based display if modules are loaded
            if (this.modules && Object.keys(this.modules).length > 0) {
                this.updateModuleServiceDisplay(svc);
            } else {
                this.ensureFallbackServiceBlock(svc);
            }

            const statusText = typeof svc.status === 'string' ? svc.status : '';
            const journalText = typeof svc.journal === 'string' ? svc.journal : '';
            if ((statusText && statusText.trim()) || (journalText && journalText.trim())) {
                this.renderServiceDetail(svc.name, {
                    status: statusText,
                    journal: journalText,
                    lines: typeof svc.lines === 'number' ? svc.lines : undefined,
                });
            }
        });
        // Note: We do NOT remove pills/blocks that temporarily disappear from the list
        // to avoid flicker. A later cleanup pass could prune truly stale entries if needed.
    }

    updateModules(modules) {
        const entries = Object.entries(modules || {});
        this.modules = {};
        this.moduleUnitMap = {};

        const normalizedEntries = entries.map(([moduleName, moduleConfig]) => {
            const normalized = this.normalizeModuleConfig(moduleName, moduleConfig);
            this.modules[moduleName] = normalized;
            const units = Array.isArray(normalized.systemd_units) ? normalized.systemd_units : [];
            units.forEach(unit => {
                if (typeof unit === 'string' && unit) {
                    this.moduleUnitMap[unit] = moduleName;
                }
            });
            return [moduleName, normalized];
        });

        if (!this.servicesLogs) {
            return;
        }

        // Track existing module section IDs
        const existingSections = new Set();
        const children = Array.from(this.servicesLogs.children || []);
        children.forEach(child => {
            if (!child) return;
            try {
                const role = child.getAttribute ? child.getAttribute('data-role') : null;
                if (role === 'module-section' && child.id) {
                    existingSections.add(child.id);
                }
            } catch (e) {
                // ignore DOM errors for detached mock nodes
            }
        });

        // Add or update module sections
        normalizedEntries.forEach(([moduleName, moduleConfig]) => {
            const moduleId = 'module-' + moduleName.replace(/[^a-zA-Z0-9_-]/g, '-');
            let section = document.getElementById(moduleId);
            if (!section) {
                this.renderModuleSection(moduleName, moduleConfig);
                section = document.getElementById(moduleId);
            } else {
                // Optionally update header/description if changed
                const title = section.querySelector('.module-title');
                if (title && title.textContent !== moduleConfig.name) title.textContent = moduleConfig.name;
                const desc = section.querySelector('.module-description');
                if (desc && desc.textContent !== moduleConfig.description) desc.textContent = moduleConfig.description;
                // Controls update: re-render only if controls changed
                const controlsContainer = section.querySelector('.module-controls');
                if (controlsContainer && JSON.stringify(controlsContainer._lastControls) !== JSON.stringify(moduleConfig.controls)) {
                    this.renderModuleControls(controlsContainer.id, moduleConfig.controls, moduleName);
                    controlsContainer._lastControls = JSON.stringify(moduleConfig.controls);
                }
            }
            existingSections.delete(moduleId);
        });

        // Remove stale module sections
        existingSections.forEach(staleId => {
            const stale = document.getElementById(staleId);
            if (!stale) return;
            try {
                if (typeof this.servicesLogs.contains === 'function') {
                    if (this.servicesLogs.contains(stale)) {
                        this.servicesLogs.removeChild(stale);
                    }
                } else if (stale.parentElement === this.servicesLogs) {
                    this.servicesLogs.removeChild(stale);
                }
            } catch (e) {
                try { this.servicesLogs.removeChild(stale); } catch (_) { /* ignore */ }
            }
        });

        // Update existing services to be grouped by modules
        Object.values(this.services).forEach(svc => {
            this.updateModuleServiceDisplay(svc);
        });
    }

    normalizeModuleConfig(moduleName, moduleConfig) {
        const cfg = Object.assign({}, moduleConfig || {});
        const prettyBase = moduleName.replace(/[_-]+/g, ' ').replace(/\s+/g, ' ').trim();
        const defaultName = prettyBase ? prettyBase.replace(/\b\w/g, ch => ch.toUpperCase()) : moduleName;
        const displayName = cfg.display || cfg.name || defaultName;
        cfg.name = displayName;
        cfg.slug = cfg.slug || moduleName;
        if (!cfg.description) {
            cfg.description = `${displayName} module`;
        }
        if (!Array.isArray(cfg.controls)) {
            cfg.controls = [];
        }
        cfg.module = moduleName;
        const units = this.moduleUnitsFor(moduleName, cfg);
        cfg.systemd_units = units;
        if (units.length > 0) {
            const preferred = typeof cfg.systemd_unit === 'string' && units.includes(cfg.systemd_unit)
                ? cfg.systemd_unit
                : units[0];
            cfg.systemd_unit = preferred;
        }
        return cfg;
    }

    moduleUnitsFor(moduleName, moduleConfig) {
        const rawUnits = [];
        if (moduleConfig) {
            const multi = moduleConfig.systemd_units;
            if (Array.isArray(multi)) {
                multi.forEach(unit => rawUnits.push(unit));
            } else if (typeof multi === 'string') {
                rawUnits.push(multi);
            }
            const single = moduleConfig.systemd_unit;
            if (typeof single === 'string') {
                rawUnits.push(single);
            }
        }
        const deduped = [];
        const seen = new Set();
        rawUnits.forEach(unit => {
            if (typeof unit !== 'string') return;
            const trimmed = unit.trim();
            if (!trimmed || seen.has(trimmed)) return;
            seen.add(trimmed);
            deduped.push(trimmed);
        });
        if (!deduped.length) {
            deduped.push(`psyched-${moduleName}.service`);
        }
        return deduped;
    }

    serviceModuleSlug(unit) {
        if (typeof unit !== 'string') return '';
        return unit.replace(/^psyched-/, '').replace(/\.service$/, '');
    }

    renderModuleControls(containerId, controls, moduleName) {
        const container = document.getElementById(containerId);
        if (!container || !controls || controls.length === 0) return;
        // Tag container with module for event handlers
        container.dataset.module = moduleName;

        const controlsHtml = controls.map(control => {
            switch (control.type) {
                case 'info':
                    return `<div class="module-control info">
                        <label>${control.label}:</label>
                        <span>${control.value}</span>
                    </div>`;
                case 'link':
                    return `<div class="module-control link">
                        <a href="${control.url}" target="_blank" class="module-link">${control.label}</a>
                    </div>`;
                case 'slider':
                    return `<div class="module-control slider">
                        <label for="${control.id}">${control.label}:</label>
                        <input type="range" id="${control.id}" min="${control.min}" max="${control.max}" 
                               value="${control.value}" step="${control.step || 1}">
                        <span>${control.value}${control.unit || ''}</span>
                    </div>`;
                case 'select':
                    const options = control.options.map(opt =>
                        `<option value="${opt.value}" ${opt.value === control.value ? 'selected' : ''}>${opt.label}</option>`
                    ).join('');
                    return `<div class="module-control select">
                        <label for="${control.id}">${control.label}:</label>
                        <select id="${control.id}">${options}</select>
                    </div>`;
                case 'toggle':
                    return `<div class="module-control toggle">
                        <label>
                            <input type="checkbox" id="${control.id}" ${control.value ? 'checked' : ''}>
                            ${control.label}
                        </label>
                    </div>`;
                case 'button':
                    const styleClass = control.style === 'danger' ? 'button-danger' : 'button-primary';
                    return `<div class="module-control button">
                        <button id="${control.id}" class="${styleClass}" data-action="${control.action}">
                            ${control.label}
                        </button>
                    </div>`;
                case 'text':
                    const inputType = control.multiline ? 'textarea' : 'input';
                    return `<div class="module-control text">
                        <label for="${control.id}">${control.label}:</label>
                        ${inputType === 'textarea'
                            ? `<textarea id="${control.id}">${control.value}</textarea>`
                            : `<input type="text" id="${control.id}" value="${control.value}">`}
                    </div>`;
                case 'display':
                    return `<div class="module-control display">
                        <label>${control.label}:</label>
                        <span id="${control.id}" class="display-value">--</span>
                        ${control.unit ? `<span class="display-unit">${control.unit}</span>` : ''}
                    </div>`;
                default:
                    return '';
            }
        }).join('');

        container.innerHTML = controlsHtml;

        // Add event listeners for interactive controls
        this.attachModuleControlEvents(containerId, controls);
    }

    attachModuleControlEvents(containerId, controls) {
        const container = document.getElementById(containerId);
        if (!container) return;
        const moduleName = container.dataset.module;

        controls.forEach(control => {
            if (!control.id) return;

            const element = document.getElementById(control.id);
            if (!element) return;

            switch (control.type) {
                case 'button':
                    element.addEventListener('click', () => {
                        this.handleModuleControlAction(control.action, control.id, element.value, moduleName);
                    });
                    break;
                case 'slider':
                    element.addEventListener('input', () => {
                        const span = element.nextElementSibling;
                        if (span) span.textContent = element.value + (control.unit || '');
                        this.handleModuleControlChange(control.id, element.value, moduleName);
                    });
                    break;
                case 'select':
                case 'toggle':
                case 'text':
                    element.addEventListener('change', () => {
                        this.handleModuleControlChange(control.id, element.value, moduleName);
                    });
                    break;
            }
        });
    }

    handleModuleControlAction(action, controlId, value, moduleName) {
        const mod = moduleName || this.getModuleFromControl(controlId);
        if (!this.websocket || !mod) return;
        try {
            const payload = { type: 'module_control', kind: 'action', module: mod, id: controlId, action, value };
            this.websocket.send(JSON.stringify(payload));
        } catch (e) {
            console.error('Failed to send module control action', e);
        }
    }

    handleModuleControlChange(controlId, value, moduleName) {
        const mod = moduleName || this.getModuleFromControl(controlId);
        if (!this.websocket || !mod) return;
        try {
            const payload = { type: 'module_control', kind: 'change', module: mod, id: controlId, value };
            this.websocket.send(JSON.stringify(payload));
        } catch (e) {
            console.error('Failed to send module control change', e);
        }
    }

    getModuleFromControl(controlId) {
        // Attempt to locate parent module section from control element
        const el = document.getElementById(controlId);
        if (!el) return null;
        const container = el.closest('.module-content');
        if (!container) return null;
        const systemdSection = container.querySelector('.module-systemd');
        const parentSection = container.parentElement; // .module-section
        if (!parentSection || !parentSection.id) return null;
        // parentSection.id is 'module-<name>'
        const m = parentSection.id.replace(/^module-/, '');
        return m || null;
    }

    updateModuleServiceDisplay(svc) {
        if (!svc || !svc.name) return;

        const unitName = svc.name;
        const moduleHint = (svc.module && typeof svc.module === 'string') ? svc.module : null;
        const moduleKey = (this.moduleUnitMap && this.moduleUnitMap[unitName]) || moduleHint || this.serviceModuleSlug(unitName);
        const moduleConfig = this.modules ? this.modules[moduleKey] : null;

        if (!moduleConfig) {
            this.ensureFallbackServiceBlock(svc);
            return;
        }

        const slug = (moduleKey && typeof moduleKey === 'string' && moduleKey.trim()) ? moduleKey : this.serviceModuleSlug(unitName);
        const moduleId = 'module-' + slug.replace(/[^a-zA-Z0-9_-]/g, '-');
        const systemdContainer = document.getElementById(moduleId + '-systemd');

        if (!systemdContainer) {
            this.ensureFallbackServiceBlock(svc);
            return;
        }

        const blockId = this.serviceId(unitName) + '-detail';
        let block = document.getElementById(blockId);
        const created = !block;
        if (!block) {
            block = this.renderServiceLogBlock(unitName);
        }
        if (block && block.parentElement !== systemdContainer) {
            systemdContainer.appendChild(block);
        }
        if (created) {
            this.requestSystemdDetail(unitName, 200);
            this.watchSystemdUnit(unitName, 200);
        }

        this.updateServiceLogControls(svc);
    }

    ensureFallbackServiceBlock(svc) {
        if (!this.servicesLogs) return;

        const blockId = this.serviceId(svc.name) + '-detail';
        let block = document.getElementById(blockId);
        const created = !block;
        if (!block) {
            block = this.renderServiceLogBlock(svc.name);
        }
        if (block && block.parentElement !== this.servicesLogs) {
            this.servicesLogs.appendChild(block);
        }
        if (created) {
            this.requestSystemdDetail(svc.name, 200);
            this.watchSystemdUnit(svc.name, 200);
        }
        this.updateServiceLogControls(svc);
    }

    updateService(svc) {
        if (!svc || !svc.name) return;
        this.services[svc.name] = svc;
        const id = this.serviceId(svc.name);
        const el = document.getElementById(id);
        if (el) {
            this.populateServicePill(el, svc);
        } else if (this.servicesContainer) {
            this.servicesContainer.appendChild(this.renderServicePill(svc));
        }
    }

    serviceId(unit) {
        return 'svc-' + unit.replace(/[^a-zA-Z0-9_-]/g, '-');
    }

    prettyServiceName(unit) {
        return unit.replace(/^psyched-/, '').replace(/\.service$/, '');
    }

    renderServicePill(svc) {
        const el = document.createElement('div');
        // semantic marker for service pill
        el.setAttribute('data-role', 'service-pill');
        el.id = this.serviceId(svc.name);
        el.title = svc.description || svc.name;
        el.innerHTML = `
            <span class="name"></span>
            <span class="state"></span>
            <span class="enabled"></span>
        `;
        this.populateServicePill(el, svc);
        this.attachPillEvents(el, svc.name);
        return el;
    }

    populateServicePill(el, svc) {
        const nameEl = el.querySelector('.name');
        const stateEl = el.querySelector('.state');
        const enEl = el.querySelector('.enabled');
        if (nameEl) nameEl.textContent = this.prettyServiceName(svc.name);
        const active = (svc.active || '').toLowerCase();
        stateEl.textContent = active || 'unknown';
        // expose the state as a data attribute for styling without inline classes
        stateEl.setAttribute('data-state', (active === 'active' ? 'active' : (active === 'inactive' ? 'inactive' : 'unknown')));
        const enabled = (svc.enabled || '').toLowerCase();
        enEl.textContent = enabled ? `(${enabled})` : '';
        // expose enabled status as data attribute
        enEl.setAttribute('data-enabled', (enabled === 'enabled' ? 'on' : 'off'));
        el.title = (svc.description || svc.name) + `\nState: ${stateEl.textContent}  Enabled: ${enabled || 'unknown'}`;
    }

    attachPillEvents(el, unit) {
        let timer = null;
        let longPressed = false;
        const start = (e) => {
            e.preventDefault();
            longPressed = false;
            clearTimeout(timer);
            timer = setTimeout(() => {
                longPressed = true;
                this.onPillLongPress(unit);
            }, 600);
        };
        const end = (e) => {
            e.preventDefault();
            clearTimeout(timer);
            if (!longPressed) this.onPillClick(unit);
        };
        el.addEventListener('mousedown', start);
        el.addEventListener('touchstart', start, { passive: false });
        el.addEventListener('mouseup', end);
        el.addEventListener('mouseleave', end);
        el.addEventListener('touchend', end);
        el.addEventListener('touchcancel', end);
        el.addEventListener('contextmenu', (e) => { e.preventDefault(); this.onPillLongPress(unit); });
        // Also support middle-click to open details quickly
        el.addEventListener('auxclick', (e) => { if (e.button === 1) this.toggleDetails(unit, true); });
    }

    onPillClick(unit) {
        const svc = this.services[unit];
        if (!svc) return;
        const action = (String(svc.active).toLowerCase() === 'active') ? 'stop' : 'start';
        this.systemdAction(action, unit);
    }

    onPillLongPress(unit) {
        const svc = this.services[unit];
        if (!svc) return;
        const action = (String(svc.enabled).toLowerCase() === 'enabled') ? 'disable' : 'enable';
        this.systemdAction(action, unit);
    }

    systemdAction(action, unit) {
        if (!this.websocket) return;
        try {
            this.websocket.send(JSON.stringify({ type: 'systemd', action, unit }));
        } catch (e) {
            console.error('systemd action failed to send', e);
        }
    }

    requestSystemdDetail(unit, lines = 200) {
        if (!this.websocket) return;
        try {
            this.websocket.send(JSON.stringify({ type: 'systemd', action: 'detail', unit, lines }));
        } catch (e) {
            console.error('systemd detail request failed', e);
        }
    }

    watchSystemdUnit(unit, lines = 200) {
        if (!this.websocket) return;
        try {
            this.websocket.send(JSON.stringify({ type: 'systemd', action: 'watch', unit, lines }));
            this.watchedUnits.add(unit);
        } catch (e) {
            console.error('systemd watch request failed', e);
        }
    }

    unwatchSystemdUnit(unit) {
        if (!this.websocket) return;
        try {
            this.websocket.send(JSON.stringify({ type: 'systemd', action: 'unwatch', unit }));
            this.watchedUnits.delete(unit);
        } catch (e) {
            console.error('systemd unwatch request failed', e);
        }
    }

    renderServiceLogBlock(unit) {
        const id = this.serviceId(unit) + '-detail';
        const wrap = document.createElement('div');
        // semantic wrapper for service logs
        wrap.setAttribute('data-role', 'service-log-block');
        wrap.id = id;
        wrap.innerHTML = `
            <div class="service-log-header">
              <h4 class="service-log-title">${this.prettyServiceName(unit)}</h4>
              <div class="service-log-controls" data-unit="${unit}">
                <button class="systemd-btn" data-action="start">Start</button>
                <button class="systemd-btn" data-action="stop">Stop</button>
                <button class="systemd-btn" data-action="enable">Enable</button>
                <button class="systemd-btn" data-action="disable">Disable</button>
              </div>
            </div>
            <div class="service-log-columns">
                <div class="service-log-col">
                    <div class="service-log-heading">systemctl status</div>
                    <pre id="${id}-status" class="service-log-pre">(loading...)</pre>
                </div>
                <div class="service-log-col">
                    <div class="service-log-heading">journalctl (last 200 lines)</div>
                    <pre id="${id}-journal" class="service-log-pre">(loading...)</pre>
                </div>
            </div>
        `;
        // Wire button events
        const controls = wrap.querySelector('.service-log-controls');
        if (controls) {
            controls.querySelectorAll('.systemd-btn').forEach(btn => {
                btn.addEventListener('click', (e) => {
                    const action = btn.getAttribute('data-action');
                    this.systemdAction(action, unit);
                });
            });
        }
        // Sticky-bottom autoscroll for logs
        const statusEl = wrap.querySelector(`#${id}-status`);
        const journalEl = wrap.querySelector(`#${id}-journal`);
        [statusEl, journalEl].forEach(logEl => {
            if (!logEl) return;
            logEl.addEventListener('wheel', function () {
                logEl._userScrolled = (logEl.scrollTop + logEl.clientHeight) < (logEl.scrollHeight - 4);
            });
        });
        wrap._scrollLogsToBottom = function () {
            [statusEl, journalEl].forEach(logEl => {
                if (logEl && !logEl._userScrolled) {
                    logEl.scrollTop = logEl.scrollHeight;
                }
            });
        };
        return wrap;
    }

    updateServiceLogControls(svc) {
        const id = this.serviceId(svc.name) + '-detail';
        const block = document.getElementById(id);
        if (!block) return;
        const isActive = (svc.active || '').toLowerCase() === 'active';
        const isEnabled = (svc.enabled || '').toLowerCase() === 'enabled';

        // Update button disabled state based on service state
        const controls = block.querySelector('.service-log-controls');
        if (controls) {
            const startBtn = controls.querySelector('[data-action="start"]');
            const stopBtn = controls.querySelector('[data-action="stop"]');
            const enableBtn = controls.querySelector('[data-action="enable"]');
            const disableBtn = controls.querySelector('[data-action="disable"]');
            if (startBtn) startBtn.disabled = isActive;
            if (stopBtn) stopBtn.disabled = !isActive;
            if (enableBtn) enableBtn.disabled = isEnabled;
            if (disableBtn) disableBtn.disabled = !isEnabled;
        }
    }

    toggleDetails(unit, open) { /* no-op after logs redesign */ }

    renderServiceDetail(unit, detail) {
        const base = this.serviceId(unit) + '-detail';
        const st = document.getElementById(base + '-status');
        const jl = document.getElementById(base + '-journal');

        // Use last non-empty values to avoid blinking into emptiness
        const prev = this.serviceDetails[unit] || { status: '', journal: '' };
        const nextStatus = (detail.status ?? '').trim();
        const nextJournal = (detail.journal ?? '').trim();

        const finalStatus = nextStatus !== '' ? nextStatus : (prev.status || '(no status output)');
        const finalJournal = nextJournal !== '' ? nextJournal : (prev.journal || '(no journal output)');

        // Helper to detect sticky bottom before update
        const isAtBottom = (el) => {
            if (!el) return false;
            const threshold = 4; // px tolerance
            return (el.scrollTop + el.clientHeight) >= (el.scrollHeight - threshold);
        };

        // Update status pane if changed; maintain sticky-bottom autoscroll
        if (st) {
            const stick = isAtBottom(st);
            if (st.textContent !== finalStatus) {
                st.textContent = finalStatus;
                if (stick) st.scrollTop = st.scrollHeight;
            }
        }

        // Update journal pane if changed; maintain sticky-bottom autoscroll
        if (jl) {
            const stick = isAtBottom(jl);
            if (jl.textContent !== finalJournal) {
                jl.textContent = finalJournal;
                if (stick) jl.scrollTop = jl.scrollHeight;
            }
        }

        // Update cache only with non-empty values
        this.serviceDetails[unit] = {
            status: nextStatus !== '' ? nextStatus : prev.status,
            journal: nextJournal !== '' ? nextJournal : prev.journal,
        };
    }

    flashServiceError(unit, msg) {
        const id = this.serviceId(unit);
        const el = document.getElementById(id);
        if (!el) return;
        // Toggle a data attribute so styling remains in CSS instead of inline
        el.setAttribute('data-error', 'true');
        el.title = (el.title || unit) + `\nError: ${msg}`;
        setTimeout(() => { el.removeAttribute('data-error'); }, 1200);
    }

    renderTopicLabel(topic, count) {
        const label = (topic || '').trim() || '--';
        if (typeof count === 'number' && Number.isFinite(count)) {
            return `${label} (${count} subscribers)`;
        }
        return label;
    }

    updateCmdVelTopic(topic, count) {
        if (this.store && typeof this.store.setCmdVelTopic === 'function') {
            this.store.setCmdVelTopic(topic, count);
        }
        const el = document.getElementById('cmdVelTopic');
        if (el) {
            el.textContent = this.renderTopicLabel(topic, count);
        }
        this.ensureCmdVelSubscription(topic);
    }

    updateVoiceTopic(topic, count) {
        if (this.store && typeof this.store.setVoiceTopic === 'function') {
            this.store.setVoiceTopic(topic, count);
        }
        const el = document.getElementById('voiceTopic');
        if (el) {
            el.textContent = this.renderTopicLabel(topic, count);
        }
    }

    updateBattery(payload) {
        // payload may include: percent, voltage, current, temperature, charging_state, charge_ratio
        const els = this.batteryEls;
        if (!els) return;

        const formatted = this.store && typeof this.store.setBattery === 'function'
            ? this.store.setBattery(payload)
            : null;

        const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
        const percent = typeof payload.percent === 'number' && isFinite(payload.percent)
            ? clamp(payload.percent, 0, 100)
            : null;

        if (els.fill && percent != null) {
            els.fill.style.width = `${percent}%`;
            // Change fill color smoothly by adjusting background position along gradient
            // Already gradient-based; width reflects SoC.
        }
        const pctText = formatted ? formatted.percent : (percent != null ? `${percent.toFixed(0)}%` : '--');
        if (els.percent) els.percent.textContent = pctText;
        if (els.percentInfo) els.percentInfo.textContent = pctText;

        if (els.voltage) {
            els.voltage.textContent = formatted ? formatted.voltage : (typeof payload.voltage === 'number' ? payload.voltage.toFixed(2) : '--');
        }
        if (els.current) {
            els.current.textContent = formatted ? formatted.current : (typeof payload.current === 'number' ? payload.current.toFixed(2) : '--');
        }
        if (els.temp) {
            els.temp.textContent = formatted ? formatted.temperature : '--';
        }

        const stateStr = formatted ? formatted.state : this.formatChargingState(payload.charging_state);
        if (els.state) els.state.textContent = stateStr;
        if (els.stateInfo) els.stateInfo.textContent = stateStr;
    }

    formatChargingState(code) {
        switch (code) {
            case 0: return 'Not charging';
            case 1: return 'Reconditioning';
            case 2: return 'Full';
            case 3: return 'Trickle';
            case 4: return 'Waiting';
            case 5: return 'Fault';
            default: return '--';
        }
    }

    updateRobotStatus(s) {
        const els = this.robotEls;
        const formatted = this.store && typeof this.store.setRobotStatus === 'function'
            ? this.store.setRobotStatus(s)
            : null;
        if (!els) return;
        const modeStr = formatted ? formatted.mode : this.formatMode(s.mode);
        if (els.mode) els.mode.textContent = modeStr;
        if (els.speed) {
            const speedText = formatted ? formatted.speed : (typeof s.speed === 'number' ? s.speed.toFixed(2) : '--');
            els.speed.textContent = speedText;
        }
        if (els.bumper) {
            const bumperText = formatted ? formatted.bumper : (typeof s.bumper === 'boolean' ? (s.bumper ? 'PRESSED' : 'OK') : '--');
            els.bumper.textContent = bumperText;
        }
        if (els.cliff) {
            const cliffText = formatted ? formatted.cliff : (typeof s.cliff === 'boolean' ? (s.cliff ? 'DETECTED' : 'OK') : '--');
            els.cliff.textContent = cliffText;
        }
        if (els.ir) {
            const irText = formatted ? formatted.ir : (typeof s.ir_omni === 'number' ? String(s.ir_omni) : '--');
            els.ir.textContent = irText;
        }
        if (els.diag) {
            const diagText = formatted ? formatted.diag : '--';
            els.diag.textContent = diagText;
        }
    }

    formatMode(code) {
        switch (code) {
            case 0: return 'OFF';
            case 1: return 'PASSIVE';
            case 2: return 'SAFE';
            case 3: return 'FULL';
            default: return '--';
        }
    }

    updateHostHealth(h) {
        const els = this.hostEls;
        const formatted = this.store && typeof this.store.setHostHealth === 'function'
            ? this.store.setHostHealth(h)
            : null;
        if (!els) return;
        if (els.cpu) {
            els.cpu.textContent = formatted ? formatted.cpu : (typeof h.cpu_percent === 'number' ? `${h.cpu_percent.toFixed(0)}%` : '--');
        }
        if (els.temp) {
            els.temp.textContent = formatted ? formatted.temp : '--';
        }
        if (els.mem) {
            els.mem.textContent = formatted ? formatted.mem : (typeof h.mem_used_percent === 'number' ? `${h.mem_used_percent.toFixed(0)}%` : '--');
        }
    }

    updateImu(imu) {
        const els = this.imuEls;
        if (!els || !els.overlay) return;

        // --- 1) Cache circle geometry & set a full-circle path once ---
        if (!this._imuGeom) {
            // Use the base circle already in your SVG so radii always match
            const baseCircle = els.overlay.querySelector('circle');
            const r = baseCircle ? parseFloat(baseCircle.getAttribute('r')) : 88;
            const cx = baseCircle ? parseFloat(baseCircle.getAttribute('cx')) : 100;
            const cy = baseCircle ? parseFloat(baseCircle.getAttribute('cy')) : 100;

            // Full circle path that starts at TOP and proceeds CLOCKWISE
            // (two 180° arcs)
            const dCircle =
                `M ${cx} ${cy - r} ` +
                `A ${r} ${r} 0 1 1 ${cx} ${cy + r} ` +
                `A ${r} ${r} 0 1 1 ${cx} ${cy - r}`;

            els.gyroZ.setAttribute('d', dCircle);

            // Cache circumference for dash math
            this._imuGeom = { cx, cy, r, C: 2 * Math.PI * r };
        }

        const { cx, cy, r, C } = this._imuGeom;

        // --- 2) Yaw (heading) rotation for the robot icon ---
        if (typeof imu.yaw === 'number' && isFinite(imu.yaw) && els.robotYaw) {
            els.robotYaw.setAttribute('transform', `rotate(${imu.yaw * 180 / Math.PI})`);
        }

        // --- 3) Acceleration vector (ax forward, ay left) ---
        if (typeof imu.ax === 'number' && typeof imu.ay === 'number' && els.accelVec) {
            const scale = 10;
            const dx = -imu.ay * scale; // left/right
            const dy = -imu.ax * scale; // up/down
            const x2 = cx + Math.max(-0.9 * r, Math.min(0.9 * r, dx));
            const y2 = cy + Math.max(-0.9 * r, Math.min(0.9 * r, dy));
            els.accelVec.setAttribute('x2', String(x2));
            els.accelVec.setAttribute('y2', String(y2));
            els.accelVec.setAttribute('opacity', (Math.hypot(dx, dy) > 2) ? '0.95' : '0.5');
        }

        // --- 4) Gyro Z (yaw rate) as a dash segment on that circle ---
        if (typeof imu.gz === 'number' && isFinite(imu.gz) && els.gyroZ) {
            const maxGz = 4.0;                 // rad/s → full half-circle
            const dead = 0.06;                 // small deadzone
            const gz = Math.max(-maxGz, Math.min(maxGz, imu.gz));
            const mag = Math.abs(gz);

            if (mag < dead) {
                els.gyroZ.style.strokeDasharray = '0 ' + C; // hide
                els.gyroZ.setAttribute('opacity', '0.25');
                return;
            }

            // We display up to 180° of arc (half the circumference)
            const sweepFrac = mag / maxGz;                 // 0..1
            const arcLen = (C / 2) * sweepFrac;            // length of visible dash
            const gapLen = Math.max(1, C - arcLen);        // remainder

            // Path is CW from TOP. For positive gz, show right-hand side (CW).
            // For negative gz, shift the dash so it appears to the LEFT of top.
            els.gyroZ.style.strokeDasharray = `${arcLen} ${gapLen}`;
            els.gyroZ.style.strokeDashoffset = (gz >= 0) ? '0' : String(-arcLen);

            els.gyroZ.setAttribute('opacity', sweepFrac > 0.05 ? '0.9' : '0.4');
        }
    }


}

if (typeof module !== 'undefined' && typeof module.exports !== 'undefined') {
    module.exports = { PilotController };
}

if (typeof window !== 'undefined') {
    window.PilotController = PilotController;
}

// Initialize the pilot controller when the page loads
document.addEventListener('DOMContentLoaded', () => {
    if (typeof window === 'undefined') {
        return;
    }
    window.PilotController = PilotController;
    // Keep a global reference for debugging and for additional handlers
    if (!window.pilotController && !window.__PILOT_SKIP_AUTO_INIT__) {
        window.pilotController = new PilotController();
    }
    const pc = window.pilotController;
    if (!pc) {
        return;
    }
    // Attach image handler to incoming websocket messages (if websocket already present it will be reused)
    try {
        // Wrap existing handleWebSocketMessage to intercept image and map messages
        const origHandle = pc.handleWebSocketMessage.bind(pc);
        pc.handleWebSocketMessage = function (data) {
            // Data may be raw string; ensure JSON parse happens once (the original expects parsed object)
            let message = data;
            if (typeof data === 'string') {
                try { message = JSON.parse(data); } catch (e) { /* non-json - ignore */ }
            }
            // If image message, update UI directly
            try {
                if (message && message.type === 'image' && message.topic && message.data) {
                    if (message.topic === '/image_raw') {
                        const el = document.getElementById('camera-image');
                        if (el) el.src = message.data;
                    } else if (message.topic === '/depth/image_raw') {
                        const el = document.getElementById('depth-image');
                        if (el) el.src = message.data;
                    } else if (message.topic === '/map' || message.topic === 'map') {
                        // Map image payload
                        if (pc.mapCtx) {
                            const img = new Image();
                            img.onload = () => {
                                const cw = pc.mapCanvas.width;
                                const ch = pc.mapCanvas.height;
                                const ar = img.width / img.height;
                                let dw = cw, dh = ch;
                                if (cw / ch > ar) { dw = ch * ar; } else { dh = cw / ar; }
                                pc.mapCtx.clearRect(0, 0, cw, ch);
                                pc.mapCtx.drawImage(img, (cw - dw) / 2, (ch - dh) / 2, dw, dh);
                            };
                            img.src = message.data;
                        }
                    }
                }
            } catch (e) {
                // ignore
            }

            // Handle raw map payload (map_raw)
            try {
                if (message && message.type === 'map_raw' && message.data) {
                    const w = parseInt(message.width, 10);
                    const h = parseInt(message.height, 10);
                    const b64 = message.data;
                    const raw = atob(b64);
                    const arr = new Uint8Array(raw.length);
                    for (let i = 0; i < raw.length; i++) arr[i] = raw.charCodeAt(i);

                    if (pc.mapCtx) {
                        const canvas = pc.mapCanvas;
                        // Resize canvas to preserve map aspect but keep reasonable size
                        const maxSize = 800;
                        let scale = 1.0;
                        if (w > h) scale = Math.min(maxSize / w, 1.0);
                        else scale = Math.min(maxSize / h, 1.0);
                        canvas.width = Math.max(200, Math.floor(w * scale));
                        canvas.height = Math.max(200, Math.floor(h * scale));
                        const imgData = pc.mapCtx.createImageData(w, h);
                        for (let y = 0; y < h; y++) {
                            for (let x = 0; x < w; x++) {
                                const idx = y * w + x;
                                let v = (arr[idx] & 0xFF) - 128;
                                let color = 127;
                                if (v === -1) color = 127;
                                else if (v === 0) color = 255;
                                else if (v > 0) color = 0;
                                const p = ((h - 1 - y) * w + x) * 4;
                                imgData.data[p] = color;
                                imgData.data[p + 1] = color;
                                imgData.data[p + 2] = color;
                                imgData.data[p + 3] = 255;
                            }
                        }
                        const off = document.createElement('canvas');
                        off.width = w; off.height = h;
                        const offCtx = off.getContext('2d');
                        offCtx.putImageData(imgData, 0, 0);
                        pc.mapCtx.clearRect(0, 0, canvas.width, canvas.height);
                        pc.mapCtx.drawImage(off, 0, 0, canvas.width, canvas.height);
                    }
                }
            } catch (e) {
                console.error('Failed to render raw map:', e);
            }

            try { origHandle(data); } catch (e) { console.error(e); }
        };
    } catch (e) {
        // ignore errors attaching handler
    }
});