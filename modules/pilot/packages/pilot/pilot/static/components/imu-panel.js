import { LitElement, html } from 'https://unpkg.com/lit@3.1.4/index.js?module';

function format(value, digits = 2) {
  return typeof value === 'number' && Number.isFinite(value) ? value.toFixed(digits) : '--';
}

const EMPTY = {
  acceleration: { x: '--', y: '--', z: '--' },
  angular: { x: '--', y: '--', z: '--' },
  orientation: { x: '--', y: '--', z: '--', w: '--' },
};

/**
 * Displays IMU telemetry in a compact grid.
 */
class PilotImuPanel extends LitElement {
  static properties = {
    data: { type: Object },
  };

  constructor() {
    super();
    this.data = null;
    this._handleEvent = (event) => {
      if (event?.detail != null) {
        this.data = event.detail;
        this.requestUpdate();
      }
    };
  }

  createRenderRoot() {
    return this;
  }

  connectedCallback() {
    super.connectedCallback();
    window.addEventListener('pilot-imu', this._handleEvent);
  }

  disconnectedCallback() {
    super.disconnectedCallback();
    window.removeEventListener('pilot-imu', this._handleEvent);
  }

  get summary() {
    if (!this.data) {
      return EMPTY;
    }
    const linear = this.data.linear_acceleration ?? this.data.linear ?? this.data.acceleration ?? {};
    const angular = this.data.angular_velocity ?? this.data.angular ?? {};
    const orientation = this.data.orientation ?? {};
    return {
      acceleration: {
        x: format(linear.x),
        y: format(linear.y),
        z: format(linear.z),
      },
      angular: {
        x: format(angular.x),
        y: format(angular.y),
        z: format(angular.z),
      },
      orientation: {
        x: format(orientation.x),
        y: format(orientation.y),
        z: format(orientation.z),
        w: format(orientation.w),
      },
    };
  }

  render() {
    const summary = this.summary;
    return html`
      <div class="imu-panel">
        <div>
          <h5>Linear Acceleration (m/s²)</h5>
          <ul>
            <li>X: ${summary.acceleration.x}</li>
            <li>Y: ${summary.acceleration.y}</li>
            <li>Z: ${summary.acceleration.z}</li>
          </ul>
        </div>
        <div>
          <h5>Angular Velocity (rad/s)</h5>
          <ul>
            <li>X: ${summary.angular.x}</li>
            <li>Y: ${summary.angular.y}</li>
            <li>Z: ${summary.angular.z}</li>
          </ul>
        </div>
        <div>
          <h5>Orientation (quaternion)</h5>
          <ul>
            <li>X: ${summary.orientation.x}</li>
            <li>Y: ${summary.orientation.y}</li>
            <li>Z: ${summary.orientation.z}</li>
            <li>W: ${summary.orientation.w}</li>
          </ul>
        </div>
      </div>
    `;
  }
}

customElements.define('pilot-imu-panel', PilotImuPanel);
