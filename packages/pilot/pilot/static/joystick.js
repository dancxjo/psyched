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
        this.updateAddressDisplay();

        // Send periodic keep-alive
        setInterval(() => this.sendPing(), 5000);
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
        this.joystickKnob.addEventListener('mousedown', this.onJoystickStart.bind(this));
        document.addEventListener('mousemove', this.onJoystickMove.bind(this));
        document.addEventListener('mouseup', this.onJoystickEnd.bind(this));

        // Touch events
        this.joystickKnob.addEventListener('touchstart', this.onJoystickStart.bind(this));
        document.addEventListener('touchmove', this.onJoystickMove.bind(this));
        document.addEventListener('touchend', this.onJoystickEnd.bind(this));

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

        event.preventDefault();
    }

    onJoystickMove(event) {
        if (!this.isDragging) return;

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

        // Calculate normalized velocities (-1 to 1)
        const normalizedX = knobX / this.maxKnobDistance;
        const normalizedY = -knobY / this.maxKnobDistance; // Invert Y for forward/backward

        // Map to cmd_vel (adjust max velocities as needed)
        const maxLinearVel = 1.0; // m/s
        const maxAngularVel = 2.0; // rad/s

        this.currentVelocity = {
            linear: {
                x: normalizedY * maxLinearVel, // Forward/backward
                y: 0.0,
                z: 0.0
            },
            angular: {
                x: 0.0,
                y: 0.0,
                z: -normalizedX * maxAngularVel // Left/right (negative for conventional turning)
            }
        };

        this.updateVelocityDisplay();
        this.sendVelocityCommand();

        event.preventDefault();
    }

    onJoystickEnd(event) {
        if (!this.isDragging) return;

        this.isDragging = false;
        this.joystick.classList.remove('dragging');

        // Return knob to center with smooth animation
        this.joystickKnob.style.transform = 'translate(-50%, -50%)';

        // Stop the robot
        this.currentVelocity = {
            linear: { x: 0, y: 0, z: 0 },
            angular: { x: 0, y: 0, z: 0 }
        };

        this.updateVelocityDisplay();
        this.sendVelocityCommand();

        event.preventDefault();
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
                        this.updateStatus(`Connected to ${message.cmd_vel_topic}`, 'connected');
                    } else {
                        console.log('Ping response received');
                    }
                    if (message.voice_topic) {
                        this.updateVoiceTopic(message.voice_topic, message.voice_subscriber_count);
                    }
                    break;
                case 'status':
                    if (message.cmd_vel_topic) {
                        this.updateCmdVelTopic(message.cmd_vel_topic, message.publisher_matched_count);
                        this.updateStatus(`Connected to ${message.cmd_vel_topic}`, 'connected');
                    }
                    if (message.voice_topic) {
                        this.updateVoiceTopic(message.voice_topic, message.voice_subscriber_count);
                    }
                    break;
                default:
                    console.log('Unknown message type:', message.type);
            }
        } catch (error) {
            console.error('Failed to parse WebSocket message:', error);
        }
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
}

// Initialize the pilot controller when the page loads
document.addEventListener('DOMContentLoaded', () => {
    new PilotController();
});