import { LitElement, html, css } from 'https://unpkg.com/lit@3.1.4/index.js?module';
import { createTopicSocket } from '/js/pilot.js';
import { surfaceStyles } from '/components/pilot-style.js';

const POSE_TOPIC = {
  topic: 'amcl_pose',
  type: 'geometry_msgs/msg/PoseWithCovarianceStamped',
};
const INITIAL_POSE_TOPIC = {
  topic: 'initialpose',
  type: 'geometry_msgs/msg/PoseWithCovarianceStamped',
};
const GOAL_TOPIC = {
  topic: 'goal_pose',
  type: 'geometry_msgs/msg/PoseStamped',
};

function toNumber(value) {
  const numeric = Number(value);
  return Number.isFinite(numeric) ? numeric : null;
}

function quaternionToYaw(orientation) {
  if (!orientation || typeof orientation !== 'object') {
    return null;
  }
  const x = toNumber(orientation.x) ?? 0;
  const y = toNumber(orientation.y) ?? 0;
  const z = toNumber(orientation.z) ?? 0;
  const w = toNumber(orientation.w) ?? 1;
  const sinyCosp = 2 * (w * z + x * y);
  const cosyCosp = 1 - 2 * (y * y + z * z);
  return Math.atan2(sinyCosp, cosyCosp);
}

function yawToQuaternion(yaw) {
  const half = yaw / 2;
  return {
    x: 0,
    y: 0,
    z: Math.sin(half),
    w: Math.cos(half),
  };
}

function radiansToDegrees(value) {
  if (value == null) {
    return null;
  }
  return value * (180 / Math.PI);
}

function formatNumber(value, precision = 2) {
  if (value == null) {
    return '—';
  }
  return Number(value).toFixed(precision);
}

function formatYawDisplay(yawRadians) {
  if (yawRadians == null) {
    return '—';
  }
  const degrees = radiansToDegrees(yawRadians);
  return `${degrees.toFixed(1)}° (${yawRadians.toFixed(2)} rad)`;
}

function buildHeaderStamp() {
  const now = Date.now();
  const sec = Math.floor(now / 1000);
  const nanosec = Math.floor((now % 1000) * 1e6);
  return { sec, nanosec };
}

function describePose(pose) {
  if (!pose) {
    return 'pose unavailable';
  }
  const parts = [];
  if (pose.x != null) {
    parts.push(`x=${pose.x.toFixed(2)} m`);
  }
  if (pose.y != null) {
    parts.push(`y=${pose.y.toFixed(2)} m`);
  }
  if (pose.yaw != null) {
    parts.push(`yaw=${radiansToDegrees(pose.yaw).toFixed(1)}°`);
  }
  return parts.length ? parts.join(', ') : 'pose unavailable';
}

/**
 * Lightweight navigation dashboard that surfaces localization feedback and
 * allows the pilot to broadcast target poses.
 *
 * Example:
 * ```html
 * <nav-dashboard></nav-dashboard>
 * ```
 */
class NavDashboard extends LitElement {
  static properties = {
    pose: { state: true },
    statusMessage: { state: true },
    statusTone: { state: true },
    initialPoseForm: { state: true },
    goalForm: { state: true },
    logEntries: { state: true },
  };

  static styles = [
    surfaceStyles,
    css`
      .surface-card__title {
        display: flex;
        align-items: center;
        gap: 0.5rem;
      }

      form {
        display: grid;
        gap: 0.75rem;
      }

      .form-row {
        display: grid;
        gap: 0.5rem;
      }

      .form-row--split {
        grid-template-columns: repeat(auto-fit, minmax(120px, 1fr));
      }

      label {
        font-size: 0.75rem;
        letter-spacing: 0.08em;
        text-transform: uppercase;
        color: var(--metric-label-color);
        display: flex;
        flex-direction: column;
        gap: 0.4rem;
      }

      input {
        background: rgba(0, 0, 0, 0.35);
        border: 1px solid var(--control-surface-border);
        border-radius: 0.5rem;
        padding: 0.55rem 0.75rem;
        color: var(--lcars-text);
        font: inherit;
      }

      input:focus {
        outline: 2px solid rgba(88, 178, 220, 0.45);
        outline-offset: 1px;
      }

      button[type='submit'] {
        justify-self: start;
      }

      .surface-actions {
        margin-top: 0.5rem;
      }

      .surface-log__meta {
        display: flex;
        gap: 0.5rem;
        font-size: 0.75rem;
        color: var(--lcars-muted);
      }
    `,
  ];

  constructor() {
    super();
    this.pose = {
      frame: 'map',
      x: null,
      y: null,
      yaw: null,
      stamp: null,
    };
    this.statusMessage = 'Awaiting localization updates…';
    this.statusTone = 'warning';
    this.initialPoseForm = {
      frame: 'map',
      x: '0.0',
      y: '0.0',
      yaw: '0.0',
    };
    this.goalForm = {
      frame: 'map',
      x: '0.0',
      y: '0.0',
      yaw: '0.0',
    };
    this.logEntries = [];
    this._sockets = [];
    this._publishers = new Map();
  }

  connectedCallback() {
    super.connectedCallback();
    this._subscribeToPose();
  }

  disconnectedCallback() {
    super.disconnectedCallback();
    this._shutdown();
  }

  render() {
    return html`
      <div class="surface-grid surface-grid--wide">
        <article class="surface-card">
          <h2 class="surface-card__title">Localization</h2>
          <p class="surface-status" data-variant=${this.statusTone}>${this.statusMessage}</p>
          <div class="surface-metric">
            <span class="surface-metric__label">Frame</span>
            <span class="surface-metric__value">${this.pose.frame}</span>
          </div>
          <div class="surface-metric">
            <span class="surface-metric__label">Position</span>
            <span class="surface-metric__value">x ${formatNumber(this.pose.x)} m · y ${formatNumber(this.pose.y)} m</span>
          </div>
          <div class="surface-metric">
            <span class="surface-metric__label">Heading</span>
            <span class="surface-metric__value">${formatYawDisplay(this.pose.yaw)}</span>
          </div>
          <div class="surface-metric">
            <span class="surface-metric__label">Stamp</span>
            <span class="surface-metric__value">${this.pose.stamp ?? '—'}</span>
          </div>
        </article>

        <article class="surface-card">
          <h2 class="surface-card__title">Command log</h2>
          <p class="surface-status">Recent pilot interactions</p>
          <ul class="surface-log">
            ${this.logEntries.length
              ? this.logEntries.map(
                  (entry) => html`
                    <li class="surface-log__entry">
                      <div class="surface-log__meta">
                        <span>${entry.time}</span>
                        <span>${entry.type}</span>
                      </div>
                      <div>${entry.message}</div>
                      ${entry.detail ? html`<pre class="surface-mono">${entry.detail}</pre>` : ''}
                    </li>
                  `,
                )
              : html`<li class="surface-muted">No commands dispatched yet.</li>`}
          </ul>
        </article>
      </div>

      <div class="surface-grid surface-grid--medium" style="margin-top: 1rem;">
        <article class="surface-card">
          <h2 class="surface-card__title">Set initial pose</h2>
          <p class="surface-status">Broadcasts to <code>${INITIAL_POSE_TOPIC.topic}</code></p>
          <form @submit=${(event) => this._submitInitialPose(event)}>
            <div class="form-row form-row--split">
              ${this._renderPoseFields(this.initialPoseForm, (field, value) => this._updateInitialPoseField(field, value))}
            </div>
            <button type="submit" class="surface-action">Set initial pose</button>
          </form>
        </article>

        <article class="surface-card">
          <h2 class="surface-card__title">Navigate to pose</h2>
          <p class="surface-status">Publishes targets on <code>${GOAL_TOPIC.topic}</code></p>
          <form @submit=${(event) => this._submitGoalPose(event)}>
            <div class="form-row form-row--split">
              ${this._renderPoseFields(this.goalForm, (field, value) => this._updateGoalField(field, value))}
            </div>
            <button type="submit" class="surface-action">Send navigation goal</button>
          </form>
        </article>
      </div>
    `;
  }

  _renderPoseFields(state, onChange) {
    return html`
      <label>
        Frame
        <input
          name="frame"
          autocomplete="off"
          .value=${state.frame}
          @input=${(event) => onChange('frame', event.target.value)}
        />
      </label>
      <label>
        X (m)
        <input
          type="number"
          step="0.01"
          name="x"
          autocomplete="off"
          .value=${state.x}
          @input=${(event) => onChange('x', event.target.value)}
          required
        />
      </label>
      <label>
        Y (m)
        <input
          type="number"
          step="0.01"
          name="y"
          autocomplete="off"
          .value=${state.y}
          @input=${(event) => onChange('y', event.target.value)}
          required
        />
      </label>
      <label>
        Yaw (deg)
        <input
          type="number"
          step="1"
          name="yaw"
          autocomplete="off"
          .value=${state.yaw}
          @input=${(event) => onChange('yaw', event.target.value)}
          required
        />
      </label>
    `;
  }

  _subscribeToPose() {
    const socket = createTopicSocket({
      topic: POSE_TOPIC.topic,
      type: POSE_TOPIC.type,
      role: 'subscribe',
    });
    socket.addEventListener('message', (event) => {
      try {
        const payload = JSON.parse(event.data);
        if (payload.event !== 'message') {
          return;
        }
        this._applyPose(payload.data);
      } catch (error) {
        console.warn('Failed to parse localization payload', error);
      }
    });
    socket.addEventListener('open', () => {
      this._setStatus('Listening for localization updates…', 'success');
    });
    socket.addEventListener('error', () => {
      this._setStatus('Localization stream unavailable', 'error');
    });
    socket.addEventListener('close', () => {
      this._setStatus('Localization stream disconnected', 'warning');
    });
    this._sockets.push(socket);
  }

  _applyPose(message) {
    if (!message || typeof message !== 'object') {
      return;
    }
    const pose = message.pose?.pose ?? {};
    const position = pose.position ?? {};
    const orientation = pose.orientation ?? {};
    const yaw = quaternionToYaw(orientation);
    const frame = message.header?.frame_id || this.pose.frame;
    const stamp = this._formatStamp(message.header?.stamp);

    this.pose = {
      frame,
      x: toNumber(position.x),
      y: toNumber(position.y),
      yaw,
      stamp,
    };

    this._setStatus('Localization active', 'success');
    this._pushLog('Localization update received', 'telemetry', describePose(this.pose));
  }

  _formatStamp(stamp) {
    if (!stamp) {
      return null;
    }
    const sec = toNumber(stamp.sec);
    const nanosec = toNumber(stamp.nanosec);
    if (sec == null) {
      return null;
    }
    const millis = sec * 1000 + Math.floor((nanosec ?? 0) / 1e6);
    try {
      return new Date(millis).toLocaleTimeString();
    } catch (_error) {
      return `${sec}.${String(nanosec ?? 0).padStart(9, '0')}`;
    }
  }

  _updateInitialPoseField(field, value) {
    this.initialPoseForm = { ...this.initialPoseForm, [field]: value };
  }

  _updateGoalField(field, value) {
    this.goalForm = { ...this.goalForm, [field]: value };
  }

  _submitInitialPose(event) {
    event.preventDefault();
    const parsed = this._parsePoseForm(this.initialPoseForm);
    if (!parsed) {
      this._setStatus('Unable to send initial pose – invalid input', 'error');
      return;
    }
    const message = {
      header: {
        frame_id: parsed.frame,
        stamp: buildHeaderStamp(),
      },
      pose: {
        pose: {
          position: { x: parsed.x, y: parsed.y, z: 0 },
          orientation: yawToQuaternion(parsed.yaw),
        },
        covariance: [
          0.25, 0, 0, 0, 0, 0,
          0, 0.25, 0, 0, 0, 0,
          0, 0, 0.25, 0, 0, 0,
          0, 0, 0, 0.25, 0, 0,
          0, 0, 0, 0, 0.25, 0,
          0, 0, 0, 0, 0, 0.0685,
        ],
      },
    };
    this._publish(INITIAL_POSE_TOPIC.topic, INITIAL_POSE_TOPIC.type, message);
    this._setStatus('Initial pose broadcast sent', 'success');
    this._pushLog('Initial pose broadcast', 'command', describePose(parsed));
  }

  _submitGoalPose(event) {
    event.preventDefault();
    const parsed = this._parsePoseForm(this.goalForm);
    if (!parsed) {
      this._setStatus('Unable to send navigation goal – invalid input', 'error');
      return;
    }
    const message = {
      header: {
        frame_id: parsed.frame,
        stamp: buildHeaderStamp(),
      },
      pose: {
        position: { x: parsed.x, y: parsed.y, z: 0 },
        orientation: yawToQuaternion(parsed.yaw),
      },
    };
    this._publish(GOAL_TOPIC.topic, GOAL_TOPIC.type, message);
    this._setStatus('Navigation goal dispatched', 'success');
    this._pushLog('Navigation goal dispatched', 'command', describePose(parsed));
  }

  _parsePoseForm(form) {
    const frame = form.frame?.trim() || this.pose.frame || 'map';
    const x = toNumber(form.x);
    const y = toNumber(form.y);
    const yawDeg = toNumber(form.yaw);
    if (x == null || y == null || yawDeg == null) {
      return null;
    }
    const yaw = yawDeg * (Math.PI / 180);
    return { frame, x, y, yaw };
  }

  _publish(topic, type, message) {
    const socket = this._ensurePublisher(topic, type);
    socket.send(message);
  }

  _ensurePublisher(topic, type) {
    const key = `${topic}:${type}`;
    if (this._publishers.has(key)) {
      return this._publishers.get(key);
    }
    const socket = createTopicSocket({ topic, type, role: 'publish' });
    this._publishers.set(key, socket);
    this._sockets.push(socket);
    return socket;
  }

  _setStatus(message, tone = 'success') {
    this.statusMessage = message;
    this.statusTone = tone;
  }

  _pushLog(message, type, detail) {
    const entry = {
      time: new Date().toLocaleTimeString(),
      type,
      message,
      detail,
    };
    this.logEntries = [entry, ...this.logEntries].slice(0, 12);
  }

  _shutdown() {
    for (const socket of this._sockets) {
      try {
        socket.close();
      } catch (_error) {
        // ignore socket closure errors to keep shutdown path resilient
      }
    }
    this._sockets = [];
    this._publishers.clear();
  }
}

customElements.define('nav-dashboard', NavDashboard);
