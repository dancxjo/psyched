import { LitElement, html, css } from 'https://unpkg.com/lit@3.1.4/index.js?module';

const MAX_LINEAR = 0.7;
const MAX_ANGULAR = 2.0;
const DEAD_ZONE = 0.06;
const SEND_INTERVAL_MS = 60;

/**
 * Two-axis joystick used to publish geometry.twist messages over the websocket.
 */
class CockpitJoystickControl extends LitElement {
  static properties = {
    record: { type: Object },
  };

  constructor() {
    super();
    this.record = null;
    this.active = false;
    this.lastSend = 0;
    this.displayLinear = 0;
    this.displayAngular = 0;
    this._container = null;
    this._knob = null;
    this._handlePointerDown = (event) => this.handlePointerDown(event);
    this._handlePointerMove = (event) => this.handlePointerMove(event);
    this._handlePointerUp = (event) => this.handlePointerUp(event);
  }

  firstUpdated() {
    this._container = this.shadowRoot.querySelector('.joystick');
    this._knob = this.shadowRoot.querySelector('.joystick-knob');
    this._container?.addEventListener('pointerdown', this._handlePointerDown);
    window.addEventListener('pointermove', this._handlePointerMove);
    window.addEventListener('pointerup', this._handlePointerUp);
  }

  static styles = [css`
    :host {
      display: block;
    }
    .joystick {
      display: flex;
      flex-direction: column;
      gap: 0.75rem;
      align-items: center;
    }
    .joystick-grid {
      position: relative;
      width: 180px;
      height: 180px;
      border-radius: 50%;
      background: radial-gradient(circle at 50% 50%, rgba(88, 178, 220, 0.25), rgba(27, 31, 42, 0.9));
      box-shadow: inset 0 0 30px rgba(0, 0, 0, 0.4);
      touch-action: none;
      overscroll-behavior: contain;
    }
    .joystick-knob {
      position: absolute;
      top: 50%;
      left: 50%;
      width: 60px;
      height: 60px;
      border-radius: 50%;
      background: radial-gradient(circle at 30% 30%, rgba(248, 128, 60, 0.8), rgba(248, 128, 60, 0.4));
      transform: translate(-50%, -50%);
      transition: transform 0.05s ease;
      cursor: grab;
    }
    .joystick-knob:active {
      cursor: grabbing;
    }
    .joystick-readout {
      display: flex;
      gap: 1rem;
      font-size: 0.8rem;
      color: var(--lcars-muted, #9aa0b5);
      font-family: 'Source Code Pro', monospace;
    }
  `];

  disconnectedCallback() {
    super.disconnectedCallback();
    this._container?.removeEventListener('pointerdown', this._handlePointerDown);
    window.removeEventListener('pointermove', this._handlePointerMove);
    window.removeEventListener('pointerup', this._handlePointerUp);
  }

  handlePointerDown(event) {
    if (event.pointerType === 'mouse' && event.button !== 0) {
      return;
    }
    if (!this._container) {
      return;
    }
    this.active = true;
    this._container.setPointerCapture(event.pointerId);
    this.updateFromEvent(event);
  }

  handlePointerMove(event) {
    if (!this.active) {
      return;
    }
    this.updateFromEvent(event);
  }

  handlePointerUp(event) {
    if (!this.active) {
      return;
    }
    this.active = false;
    try {
      this._container?.releasePointerCapture(event.pointerId);
    } catch (error) {
      // Ignore release failures
    }
    this.resetKnob();
  }

  clamp(value, min, max) {
    return Math.min(Math.max(value, min), max);
  }

  sendVelocity(linearX, angularZ) {
    const now = Date.now();
    if (now - this.lastSend < SEND_INTERVAL_MS) {
      return;
    }
    this.lastSend = now;
    this.displayLinear = linearX;
    this.displayAngular = angularZ;
    this.record?.send?.({
      linear: { x: linearX, y: 0.0, z: 0.0 },
      angular: { x: 0.0, y: 0.0, z: angularZ },
    });
    this.requestUpdate();
  }

  resetKnob() {
    if (this._knob) {
      this._knob.style.transform = 'translate(-50%, -50%)';
    }
    this.displayLinear = 0;
    this.displayAngular = 0;
    this.record?.send?.({
      linear: { x: 0.0, y: 0.0, z: 0.0 },
      angular: { x: 0.0, y: 0.0, z: 0.0 },
    });
    this.requestUpdate();
  }

  updateFromEvent(event) {
    if (!this._container || !this._knob) {
      return;
    }
    const rect = this._container.getBoundingClientRect();
    const centerX = rect.left + rect.width / 2;
    const centerY = rect.top + rect.height / 2;
    const radius = Math.min(rect.width, rect.height) / 2;
    const knobRadius = this._knob.offsetWidth / 2;
    const maxDistance = radius - knobRadius;

    const offsetX = event.clientX - centerX;
    const offsetY = event.clientY - centerY;

    const distance = Math.min(Math.hypot(offsetX, offsetY), maxDistance);
    const angle = Math.atan2(offsetY, offsetX);

    const knobX = Math.cos(angle) * distance;
    const knobY = Math.sin(angle) * distance;

    this._knob.style.transform = `translate(calc(-50% + ${knobX}px), calc(-50% + ${knobY}px))`;

    const nx = knobX / maxDistance;
    const ny = knobY / maxDistance;
    const filteredX = Math.abs(nx) < DEAD_ZONE ? 0 : this.clamp(nx, -1, 1);
    const filteredY = Math.abs(ny) < DEAD_ZONE ? 0 : this.clamp(ny, -1, 1);

    const linearX = -filteredY * MAX_LINEAR;
    const angularZ = -filteredX * MAX_ANGULAR;
    this.sendVelocity(linearX, angularZ);
  }

  render() {
    return html`
      <div class="joystick">
        <div class="joystick-grid">
          <div class="joystick-knob"></div>
        </div>
        <div class="joystick-readout">
          <span>Linear X: ${this.displayLinear.toFixed(2)} m/s</span>
          <span>Angular Z: ${this.displayAngular.toFixed(2)} rad/s</span>
        </div>
      </div>
    `;
  }
}

customElements.define('cockpit-joystick-control', CockpitJoystickControl);
