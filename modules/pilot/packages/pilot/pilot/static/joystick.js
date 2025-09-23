/**
 * Pilot Joystick Control Interface
 * Handles joystick interaction and WebSocket communication for robot control
 */

class PilotController {
    constructor(options = {}) {
        const opts = options || {};
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
        this.sendRateMs = 50; // Send at most every 50ms (20 Hz)

        // Systemd/services state defaults (will be hydrated in init())
        this.services = {};
        this.servicesContainer = null;
        this.servicesLogs = null;
        this.serviceDetails = {};
        this.watchedUnits = new Set();
        this.modules = {};
        this.moduleUnitMap = {};

        if (!opts.deferInit) {
            this.init();
        }
    }

    init() {
        this.setupWebSocket();
        this.setupJoystick();
        this.setupButtons();
        this.setupVoice();
        // Throttle slider removed; use D-Pad instead
        this.setupDpad();
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
        this.services = {}; // map unit -> { name, active, enabled, description }
        this.servicesContainer = document.getElementById('servicesPills');
        this.servicesLogs = document.getElementById('servicesLogs');
        // Cache of last non-empty service details to prevent blink/empty overwrites
        this.serviceDetails = {}; // unit -> { status: string, journal: string }
        // Track watched units to allow cleanup if needed
        this.watchedUnits = new Set();

        // Modules UI
        this.modules = {}; // map module_name -> module config
        this.moduleUnitMap = {}; // unit name -> module key

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
            } catch (_) { }
        });
    }

    setupHostHealthToggle() {
        const panel = document.getElementById('hostHealthPanel');
        const toggle = document.getElementById('hostHealthToggle');
        if (!panel || !toggle) return;
        const small = window.matchMedia('(max-width: 640px)');
        const applyDefault = () => {
            const collapsed = small.matches; // default collapsed on small screens
            panel.classList.toggle('collapsed', collapsed);
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
            const collapsed = panel.classList.toggle('collapsed');
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
                depthEl.style.display = toggle.checked ? 'block' : 'none';
            });
        }
        if (slider) {
            const setOpacity = (v) => {
                const op = Math.max(0, Math.min(100, parseInt(v, 10))) / 100.0;
                depthEl.style.opacity = String(op);
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
                    this.websocket.send(JSON.stringify({ type: 'save_map', name: 'rtabmap_map' }));
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
        document.getElementById('webAddress').textContent = webAddress;
        document.getElementById('wsAddress').textContent = wsAddress;
    }

    updateWsAddress(url) {
        const el = document.getElementById('wsAddress');
        if (el) el.textContent = url || '—';
    }

    onJoystickStart(event) {
        this.isDragging = true;
        this.joystick.classList.add('dragging');

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
        this.joystick.classList.remove('dragging');

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
        const up = document.getElementById('dpadUp');
        const down = document.getElementById('dpadDown');
        const left = document.getElementById('dpadLeft');
        const right = document.getElementById('dpadRight');
        if (!up || !down || !left || !right) return;

        // State of pressed buttons
        this.dpadState = { up: false, down: false, left: false, right: false };
        this.dpadLoopId = null;

        const setPressed = (key, pressed) => {
            this.dpadState[key] = pressed;
            if (pressed) {
                this.startDpadLoop();
            } else if (!this.dpadState.up && !this.dpadState.down && !this.dpadState.left && !this.dpadState.right) {
                this.stopDpadLoop();
                // When released all, stop motion
                this.currentVelocity = {
                    linear: { x: 0, y: 0, z: 0 },
                    angular: { x: 0, y: 0, z: 0 }
                };
                this.updateVelocityDisplay();
                this.sendVelocityCommand();
            }
        };

        const makeHandlers = (key) => ({
            down: (e) => { e.preventDefault(); setPressed(key, true); },
            up: (e) => { e.preventDefault(); setPressed(key, false); }
        });

        const u = makeHandlers('up');
        const d = makeHandlers('down');
        const l = makeHandlers('left');
        const r = makeHandlers('right');

        // Mouse
        up.addEventListener('mousedown', u.down);
        down.addEventListener('mousedown', d.down);
        left.addEventListener('mousedown', l.down);
        right.addEventListener('mousedown', r.down);
        document.addEventListener('mouseup', (e) => {
            u.up(e); d.up(e); l.up(e); r.up(e);
        });
        // Touch
        up.addEventListener('touchstart', u.down, { passive: false });
        down.addEventListener('touchstart', d.down, { passive: false });
        left.addEventListener('touchstart', l.down, { passive: false });
        right.addEventListener('touchstart', r.down, { passive: false });
        const touchEndAll = (e) => { u.up(e); d.up(e); l.up(e); r.up(e); };
        document.addEventListener('touchend', touchEndAll, { passive: false });
        document.addEventListener('touchcancel', touchEndAll, { passive: false });
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

    updateVelocityDisplay() {
        document.getElementById('linearX').textContent = this.currentVelocity.linear.x.toFixed(2);
        document.getElementById('linearY').textContent = this.currentVelocity.linear.y.toFixed(2);
        document.getElementById('angularZ').textContent = this.currentVelocity.angular.z.toFixed(2);
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
        this.joystick.classList.remove('dragging');
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
        const statusText = document.getElementById('statusText');
        const statusIndicator = document.getElementById('statusIndicator');
        const status = document.getElementById('status');

        statusText.textContent = text;
        status.className = `status ${className}`;

        switch (className) {
            case 'connected':
                statusIndicator.textContent = '🟢';
                break;
            case 'disconnected':
                statusIndicator.textContent = '🔴';
                break;
            default:
                statusIndicator.textContent = '🟡';
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
                default:
                    console.log('Unknown message type:', message.type);
            }
        } catch (error) {
            console.error('Failed to parse WebSocket message:', error);
        }
    }

    addConversation(msg) {
        if (!this.convLog) return;
        const el = document.createElement('div');
        el.className = 'conversation-entry';
        const role = (msg.role || '').toString().trim() || 'assistant';
        const content = (msg.content || '').toString();
        el.innerHTML = `<span class="role">${role}:</span> <span class="content"></span>`;
        const c = el.querySelector('.content');
        if (c) c.textContent = content;
        this.convLog.appendChild(el);
        // autoscroll to bottom
        this.convLog.scrollTop = this.convLog.scrollHeight;
    }

    updateAudioStatus(a) {
        const els = this.audioEls;
        if (!els) return;
        if (typeof a.autophony_ms === 'number' && els.speakingMs) {
            els.speakingMs.textContent = String(a.autophony_ms);
        }
        if (typeof a.speech_ms === 'number' && els.vadMs) {
            els.vadMs.textContent = String(a.speech_ms);
        }
        if (a.mic) this.updateMicInfo(a.mic);
    }

    updateMicInfo(m) {
        const el = this.audioEls?.micInfo;
        if (!el) return;
        const sr = (typeof m.sample_rate === 'number' && m.sample_rate) ? `${m.sample_rate} Hz` : '--';
        const ch = (typeof m.channels === 'number' && m.channels) ? `${m.channels} ch` : '--';
        el.textContent = `${sr}, ${ch}`;
    }

    updateGps(g) {
        const els = this.gpsEls;
        if (!els) return;
        const statusStr = this.formatGpsStatus(g.status);
        if (els.fix) els.fix.textContent = statusStr;
        if (typeof g.lat === 'number' && els.lat) els.lat.textContent = g.lat.toFixed(6);
        if (typeof g.lon === 'number' && els.lon) els.lon.textContent = g.lon.toFixed(6);
        if (typeof g.alt === 'number' && els.alt) els.alt.textContent = g.alt.toFixed(1);
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
            if (!child || !child.classList) return;
            try {
                if (child.classList.contains('module-section') && child.id) {
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

    renderModuleSection(moduleName, moduleConfig) {
        if (!this.servicesLogs) return;

        const moduleId = 'module-' + moduleName.replace(/[^a-zA-Z0-9_-]/g, '-');

        // Create module container
        const moduleSection = document.createElement('div');
        moduleSection.className = 'module-section';
        moduleSection.id = moduleId;

        moduleSection.innerHTML = `
            <div class="module-header">
                <h3 class="module-title">${moduleConfig.name}</h3>
                <div class="module-description">${moduleConfig.description}</div>
            </div>
            <div class="module-content">
                <div class="module-controls" id="${moduleId}-controls">
                    <!-- Module controls will be rendered here -->
                </div>
                <div class="module-systemd" id="${moduleId}-systemd">
                    <!-- Service logs and controls will be rendered here -->
                </div>
            </div>
        `;

        this.servicesLogs.appendChild(moduleSection);

        // Render module controls
        this.renderModuleControls(moduleId + '-controls', moduleConfig.controls, moduleName);
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
        el.className = 'service-pill';
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
        stateEl.className = 'state ' + (active === 'active' ? 'active' : (active === 'inactive' ? 'inactive' : 'unknown'));
        const enabled = (svc.enabled || '').toLowerCase();
        enEl.textContent = enabled ? `(${enabled})` : '';
        enEl.className = 'enabled ' + (enabled === 'enabled' ? 'on' : 'off');
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
        wrap.className = 'service-log-block';
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
        const old = el.style.outline;
        el.style.outline = '2px solid #ef4444';
        el.title = (el.title || unit) + `\nError: ${msg}`;
        setTimeout(() => { el.style.outline = old || ''; }, 1200);
    }

    updateCmdVelTopic(topic, count) {
        const el = document.getElementById('cmdVelTopic');
        if (el) {
            el.textContent = topic + (typeof count === 'number' ? ` (${count} subscribers)` : '');
        }
    }

    updateVoiceTopic(topic, count) {
        const el = document.getElementById('voiceTopic');
        if (el) {
            el.textContent = topic + (typeof count === 'number' ? ` (${count} subscribers)` : '');
        }
    }

    updateBattery(payload) {
        // payload may include: percent, voltage, current, temperature, charging_state, charge_ratio
        const els = this.batteryEls;
        if (!els) return;

        const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
        const percent = typeof payload.percent === 'number' && isFinite(payload.percent)
            ? clamp(payload.percent, 0, 100)
            : null;

        if (els.fill && percent != null) {
            els.fill.style.width = `${percent}%`;
            // Change fill color smoothly by adjusting background position along gradient
            // Already gradient-based; width reflects SoC.
        }
        const pctText = percent != null ? `${percent.toFixed(0)}%` : '--%';
        if (els.percent) els.percent.textContent = pctText;
        if (els.percentInfo) els.percentInfo.textContent = pctText;

        if (els.voltage && typeof payload.voltage === 'number') {
            els.voltage.textContent = payload.voltage.toFixed(2);
        }
        if (els.current && typeof payload.current === 'number') {
            els.current.textContent = payload.current.toFixed(2);
        }
        if (els.temp && typeof payload.temperature === 'number') {
            const c = payload.temperature;
            const f = c * 9 / 5 + 32;
            els.temp.textContent = `${c.toFixed(1)}°C / ${f.toFixed(1)}°F`;
        }

        const stateStr = this.formatChargingState(payload.charging_state);
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
        if (!els) return;
        const modeStr = this.formatMode(s.mode);
        if (els.mode) els.mode.textContent = modeStr;
        if (els.speed && typeof s.speed === 'number') els.speed.textContent = s.speed.toFixed(2);
        if (els.bumper && typeof s.bumper === 'boolean') els.bumper.textContent = s.bumper ? 'PRESSED' : 'OK';
        if (els.cliff && typeof s.cliff === 'boolean') els.cliff.textContent = s.cliff ? 'DETECTED' : 'OK';
        if (els.ir && typeof s.ir_omni === 'number') els.ir.textContent = String(s.ir_omni);
        if (els.diag && s.diag_counts) {
            const lvl = s.diag_level ?? 0;
            const counts = s.diag_counts;
            const txt = `L${lvl} ok:${counts[0] || 0} warn:${counts[1] || 0} err:${counts[2] || 0}`;
            els.diag.textContent = txt;
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
        if (!els) return;
        if (typeof h.cpu_percent === 'number') {
            els.cpu.textContent = `CPU ${h.cpu_percent.toFixed(0)}%`;
        }
        if (typeof h.temp_c === 'number') {
            const c = h.temp_c;
            const f = c * 9 / 5 + 32;
            els.temp.textContent = `Temp ${c.toFixed(0)}°C / ${f.toFixed(0)}°F`;
        }
        if (typeof h.mem_used_percent === 'number') {
            els.mem.textContent = `Mem ${h.mem_used_percent.toFixed(0)}%`;
        }
    }

    updateImu(imu) {
        // imu: { ax, ay, az, gx, gy, gz, yaw }
        const els = this.imuEls;
        if (!els || !els.overlay) return;

        // Rotate robot by yaw (radians to degrees)
        if (typeof imu.yaw === 'number' && isFinite(imu.yaw) && els.robotYaw) {
            const deg = imu.yaw * 180 / Math.PI;
            els.robotYaw.setAttribute('transform', `rotate(${deg})`);
        }

        // Acceleration vector in plane (x: forward, y: left). Scale visually.
        if (typeof imu.ax === 'number' && typeof imu.ay === 'number' && els.accelVec) {
            const scale = 10; // pixels per m/s^2 (tuned visually)
            // In our world, +x forward = upward in joystick SVG (y decreases). +y left = left (x decreases)
            const dx = -imu.ay * scale; // left/right
            const dy = -imu.ax * scale; // up/down
            const x2 = 100 + Math.max(-80, Math.min(80, dx));
            const y2 = 100 + Math.max(-80, Math.min(80, dy));
            els.accelVec.setAttribute('x2', String(x2));
            els.accelVec.setAttribute('y2', String(y2));
            els.accelVec.setAttribute('opacity', (Math.hypot(dx, dy) > 2) ? '0.95' : '0.5');
        }

        // Gyro Z arc: map gz rad/s to arc length and direction
        if (typeof imu.gz === 'number' && isFinite(imu.gz) && els.gyroZ) {
            const maxGz = 4.0; // rad/s corresponding to full semicircle
            const clamped = Math.max(-maxGz, Math.min(maxGz, imu.gz));
            const frac = Math.abs(clamped) / maxGz; // 0..1
            const sweep = Math.max(0.02, Math.PI * frac); // up to 180 degrees

            // Draw arc centered at top (start angle depends on sign)
            const r = 88;
            const cx = 100, cy = 100;
            const sign = clamped >= 0 ? 1 : -1; // CCW positive
            // Base angle is -90deg; arc goes left for positive, right for negative
            const start = (-Math.PI / 2) + (sign > 0 ? -sweep : 0);
            const end = (-Math.PI / 2) + (sign > 0 ? 0 : sweep);
            const x0 = cx + r * Math.cos(start);
            const y0 = cy + r * Math.sin(start);
            const x1 = cx + r * Math.cos(end);
            const y1 = cy + r * Math.sin(end);
            const largeArc = sweep > Math.PI ? 1 : 0;
            const sweepFlag = sign > 0 ? 1 : 0; // CCW for positive
            const d = `M ${x0.toFixed(1)} ${y0.toFixed(1)} A ${r} ${r} 0 ${largeArc} ${sweepFlag} ${x1.toFixed(1)} ${y1.toFixed(1)}`;
            els.gyroZ.setAttribute('d', d);
            els.gyroZ.setAttribute('opacity', frac > 0.05 ? '0.9' : '0.3');
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