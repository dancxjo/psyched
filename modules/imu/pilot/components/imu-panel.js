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
    const sections = [
      {
        title: 'Linear Acceleration (m/s²)',
        entries: [
          { label: 'X', value: summary.acceleration.x },
          { label: 'Y', value: summary.acceleration.y },
          { label: 'Z', value: summary.acceleration.z },
        ],
      },
      {
        title: 'Angular Velocity (rad/s)',
        entries: [
          { label: 'X', value: summary.angular.x },
          { label: 'Y', value: summary.angular.y },
          { label: 'Z', value: summary.angular.z },
        ],
      },
      {
        title: 'Orientation (quaternion)',
        entries: [
          { label: 'X', value: summary.orientation.x },
          { label: 'Y', value: summary.orientation.y },
          { label: 'Z', value: summary.orientation.z },
          { label: 'W', value: summary.orientation.w },
        ],
      },
    ];

    return html`
      <div class="control-surface metric-grid imu-panel" data-columns="wide">
        ${sections.map(
          (section) => html`
            <div class="metric-card">
              <h5 class="metric-title audio-oscilloscope">${section.title}</h5>
              <ul class="metric-list">
                ${section.entries.map(
                  (entry) => html`
                    <li class="metric-pair">
                      <span class="metric-label">${entry.label}</span>
                      <span class="metric-value">${entry.value}</span>
                    </li>
                  `,
                )}
              </ul>
            </div>
          `,
        )}
      </div>
    `;
  }
}

customElements.define('pilot-imu-panel', PilotImuPanel);
