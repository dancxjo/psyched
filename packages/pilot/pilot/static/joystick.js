/**
 * Pilot Joystick Control Interface
 * Handles joystick interaction and WebSocket communication for robot control
 */

class PilotController {
    constructor() {
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

        this.init();
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

        // Host health toggle setup
        this.setupHostHealthToggle();

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
                    // Send immediate ping to fetch status
                    this.sendPing();
                    // Ask for services list
                    this.requestSystemdList();
                };

                this.websocket.onclose = () => {
                    const wasConnected = this.isConnected;
                    this.isConnected = false;
                    this.updateStatus('Disconnected', 'disconnected');
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

        stopButton.addEventListener('click', () => {
            this.emergencyStop();
        });

        resetButton.addEventListener('click', () => {
            this.resetJoystick();
        });
    }

    setupVoice() {
        const input = document.getElementById('voiceInput');
        const button = document.getElementById('voiceSendButton');
        const marco = document.getElementById('marcoButton');
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
    }

    updateAddressDisplay() {
        const webAddress = `${window.location.protocol}//${window.location.host}`;
        const wsPort = parseInt(window.location.port) + 1;
        const wsProtocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const wsAddress = `${wsProtocol}//${window.location.hostname}:${wsPort}`;

        document.getElementById('webAddress').textContent = webAddress;
        document.getElementById('wsAddress').textContent = wsAddress;
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
                    if (Array.isArray(message.systemd_services)) {
                        this.updateServicesList(message.systemd_services);
                    }
                    break;
                case 'status':
                    if (message.cmd_vel_topic) {
                        this.updateCmdVelTopic(message.cmd_vel_topic, message.publisher_matched_count);
                        this.updateStatus(`Connected`, 'connected');
                    }
                    if (message.voice_topic) {
                        this.updateVoiceTopic(message.voice_topic, message.voice_subscriber_count);
                    }
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
                    if (Array.isArray(message.systemd_services)) {
                        this.updateServicesList(message.systemd_services);
                    }
                    break;
                case 'systemd':
                    if (Array.isArray(message.services)) {
                        this.updateServicesList(message.services);
                    } else if (message.unit && message.status) {
                        this.updateService(message.status);
                        if (message.error) this.flashServiceError(message.unit, message.error);
                    } else if (message.error) {
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
                default:
                    console.log('Unknown message type:', message.type);
            }
        } catch (error) {
            console.error('Failed to parse WebSocket message:', error);
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
        if (!this.servicesContainer) return;
        // Merge/update map
        services.forEach(svc => {
            if (!svc || !svc.name) return;
            this.services[svc.name] = svc;
        });
        // Render all known services
        this.servicesContainer.innerHTML = '';
        Object.values(this.services).forEach(svc => {
            const el = this.renderServicePill(svc);
            this.servicesContainer.appendChild(el);
        });
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

// Initialize the pilot controller when the page loads
document.addEventListener('DOMContentLoaded', () => {
    new PilotController();
});