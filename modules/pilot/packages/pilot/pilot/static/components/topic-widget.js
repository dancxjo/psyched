import { LitElement, html, nothing } from 'https://unpkg.com/lit@3.1.4/index.js?module';

import './joystick-control.js';
import './imu-panel.js';
import './voice-console.js';
import './conversation-console.js';

const MAX_PREVIEW = 1200;

function formatNumber(value, digits = 2) {
  return typeof value === 'number' && Number.isFinite(value) ? value.toFixed(digits) : '--';
}

function describeImage(data) {
  if (!data) return 'Awaiting frame…';
  const width = data.width ?? data.info?.width;
  const height = data.height ?? data.info?.height;
  const encoding = data.encoding ?? data.info?.encoding ?? 'unknown';
  const step = data.step ?? data.info?.step;
  return `Resolution: ${width} × ${height}\nEncoding: ${encoding}\nStep: ${step ?? '--'}`;
}

function describeMap(data) {
  if (!data) return 'Awaiting occupancy grid…';
  const width = data.info?.width ?? data.width;
  const height = data.info?.height ?? data.height;
  const resolution = data.info?.resolution;
  return `Size: ${width} × ${height}\nResolution: ${formatNumber(resolution, 3)} m/px`;
}

function describePath(data) {
  if (!data) return 'Awaiting path…';
  const poses = data.poses ?? [];
  return `Waypoints: ${poses.length}`;
}

function describePose(data) {
  if (!data) return 'Awaiting pose…';
  const position = data.pose?.pose?.position ?? data.pose?.position ?? data.position ?? {};
  const orientation = data.pose?.pose?.orientation ?? data.pose?.orientation ?? data.orientation ?? {};
  return `Position: (${formatNumber(position.x)}, ${formatNumber(position.y)}, ${formatNumber(position.z)})\n` +
    `Orientation: (${formatNumber(orientation.x)}, ${formatNumber(orientation.y)}, ${formatNumber(orientation.z)}, ${formatNumber(orientation.w)})`;
}

function describeVector(data) {
  if (!data) return 'Awaiting twist…';
  const linear = data.twist?.twist?.linear ?? data.twist?.linear ?? data.linear ?? {};
  const angular = data.twist?.twist?.angular ?? data.twist?.angular ?? data.angular ?? {};
  return `Linear: (${formatNumber(linear.x)}, ${formatNumber(linear.y)}, ${formatNumber(linear.z)})\n` +
    `Angular: (${formatNumber(angular.x)}, ${formatNumber(angular.y)}, ${formatNumber(angular.z)})`;
}

function describeText(data) {
  if (data == null) return 'Awaiting message…';
  if (typeof data === 'string') return data;
  if (typeof data?.data === 'string') return data.data;
  return JSON.stringify(data, null, 2);
}

function describeGeneric(data) {
  if (data == null) return 'Awaiting data…';
  const text = typeof data === 'string' ? data : JSON.stringify(data, null, 2);
  return text.length > MAX_PREVIEW ? `${text.slice(0, MAX_PREVIEW)}…` : text;
}

/**
 * Renders a preview of topic data and contextual controls.
 */
class PilotTopicWidget extends LitElement {
  static properties = {
    record: { type: Object },
    topic: { type: Object },
  };

  constructor() {
    super();
    this.record = null;
    this.topic = null;
  }

  createRenderRoot() {
    return this;
  }

  get status() {
    return this.record?.state ?? 'idle';
  }

  get display() {
    if (!this.record) {
      return null;
    }
    const presentation = this.topic?.presentation || '';
    if (presentation === 'image' || presentation === 'depth') return describeImage(this.record.last);
    if (presentation === 'map') return describeMap(this.record.last);
    if (presentation === 'path') return describePath(this.record.last);
    if (presentation === 'pose') return describePose(this.record.last);
    if (presentation === 'vector') return describeVector(this.record.last);
    if (presentation === 'status' || presentation === 'text' || presentation === 'log') return describeText(this.record.last);
    return describeGeneric(this.record.last);
  }

  renderContent() {
    if (!this.record) {
      return html`<div class="inactive">Not subscribed</div>`;
    }
    if (this.topic?.presentation === 'joystick') {
      return html`<pilot-joystick-control .record=${this.record}></pilot-joystick-control>`;
    }
    if (this.topic?.presentation === 'imu') {
      return html`<pilot-imu-panel .data=${this.record.last}></pilot-imu-panel>`;
    }
    if (this.topic?.presentation === 'chat' || this.topic?.topic === '/conversation') {
      return html`<pilot-conversation-console .record=${this.record}></pilot-conversation-console>`;
    }
    if (this.topic?.presentation === 'voice' || this.topic?.topic === '/voice') {
      return html`<pilot-voice-console .record=${this.record}></pilot-voice-console>`;
    }
    return html`<pre class="topic-payload">${this.display}</pre>`;
  }

  render() {
    return html`
      <div class="topic-widget">
        <div class="topic-state">
          <span class=${`state ${this.status}`}>${this.status}</span>
          ${this.record?.paused ? html`<span class="paused">Paused</span>` : nothing}
          ${this.record?.error ? html`<span class="error">${this.record.error}</span>` : nothing}
        </div>
        ${this.renderContent()}
      </div>
    `;
  }
}

customElements.define('pilot-topic-widget', PilotTopicWidget);
