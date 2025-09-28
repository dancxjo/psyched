import { LitElement, html, nothing } from 'https://unpkg.com/lit@3.1.4/index.js?module';

import './joystick-control.js';
import './imu-panel.js';
import './voice-console.js';
import './conversation-console.js';
import './audio-waveform.js';
import './audio-oscilloscope.js';
import './transcription-panel.js';
import './value-gauge.js';
import './battery-panel.js';
import './battery-feed.js';
import './diagnostics-panel.js';
import './event-log.js';
import './voice-control-bridge.js';
import './voice-volume.js';

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

function formatPercent(v, digits = 1) {
  if (v == null || Number.isNaN(v)) return '--';
  if (!Number.isFinite(v)) return '--';
  return `${v.toFixed(digits)}%`;
}

function formatNumberShort(v, digits = 1) {
  if (v == null || Number.isNaN(v)) return '--';
  if (!Number.isFinite(v)) return '--';
  return v.toFixed(digits);
}

function formatUptime(sec) {
  if (sec == null || Number.isNaN(sec) || !Number.isFinite(sec)) return '--';
  sec = Math.floor(sec);
  const days = Math.floor(sec / 86400);
  sec -= days * 86400;
  const hrs = Math.floor(sec / 3600);
  sec -= hrs * 3600;
  const mins = Math.floor(sec / 60);
  const s = sec - mins * 60;
  if (days > 0) return `${days}d ${hrs}h ${mins}m`;
  if (hrs > 0) return `${hrs}h ${mins}m`;
  if (mins > 0) return `${mins}m ${s}s`;
  return `${s}s`;
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
    if (this.topic?.presentation === 'waveform') {
      return html`<pilot-audio-waveform .record=${this.record} .topic=${this.topic}></pilot-audio-waveform>`;
    }
    if (this.topic?.presentation === 'oscilloscope') {
      return html`<pilot-audio-oscilloscope .record=${this.record} .topic=${this.topic}></pilot-audio-oscilloscope>`;
    }
    if (this.topic?.presentation === 'transcription') {
      return html`<pilot-transcription-panel .record=${this.record}></pilot-transcription-panel>`;
    }
    if (this.topic?.presentation === 'gauge') {
      return html`<pilot-value-gauge .record=${this.record} .topic=${this.topic}></pilot-value-gauge>`;
    }
    if (this.topic?.presentation === 'battery-panel') {
      return html`<pilot-battery-panel .record=${this.record} .topic=${this.topic}></pilot-battery-panel>`;
    }
    if (this.topic?.presentation === 'battery-feed') {
      return html`<pilot-battery-feed .record=${this.record} .topic=${this.topic}></pilot-battery-feed>`;
    }
    if (this.topic?.presentation === 'diagnostics') {
      return html`<pilot-diagnostics-panel .record=${this.record}></pilot-diagnostics-panel>`;
    }
    if (this.topic?.presentation === 'event-log') {
      return html`<pilot-event-log .record=${this.record}></pilot-event-log>`;
    }
    if (this.topic?.presentation === 'diode') {
      // Expect boolean payloads or objects with a boolean `data` field.
      const val = (typeof this.record.last === 'boolean') ? this.record.last : (this.record.last?.data ?? false);
      const cls = `diode ${val ? 'flashing' : 'off'}`;
      return html`<div class="diode-container"><div class=${cls} title="${val ? 'Active' : 'Inactive'}"></div></div>`;
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
    if (this.topic?.presentation === 'voice-control') {
      return html`<pilot-voice-control-bridge .record=${this.record} .topic=${this.topic}></pilot-voice-control-bridge>`;
    }
    if (this.topic?.presentation === 'voice-volume') {
      return html`<pilot-voice-volume .record=${this.record}></pilot-voice-volume>`;
    }
    // Render a pleasant host health panel for the HostHealth message type
    if (this.topic?.type === 'psyched_msgs/msg/HostHealth' || (this.topic?.topic || '').startsWith('/hosts/health')) {
      const d = this.record.last || {};
      const cpu = formatPercent(d.cpu_percent);
      const load1 = formatNumberShort(d.load_avg_1);
      const load5 = formatNumberShort(d.load_avg_5);
      const load15 = formatNumberShort(d.load_avg_15);
      const memPerc = formatPercent(d.mem_used_percent);
      const memUsed = d.mem_used_mb == null || Number.isNaN(d.mem_used_mb) ? '--' : `${formatNumberShort(d.mem_used_mb)} MB`;
      const memTotal = d.mem_total_mb == null || Number.isNaN(d.mem_total_mb) ? '--' : `${formatNumberShort(d.mem_total_mb)} MB`;
      const disk = formatPercent(d.disk_used_percent_root);
      const temp = d.temp_c == null || Number.isNaN(d.temp_c) ? '--' : `${formatNumberShort(d.temp_c)} °C`;
      const uptime = formatUptime(d.uptime_sec);

      return html`
        <div class="host-health">
          <div class="row"><div class="label">CPU</div><div class="value">${cpu}</div></div>
          <div class="row"><div class="label">Load (1/5/15)</div><div class="value">${load1} / ${load5} / ${load15}</div></div>
          <div class="row"><div class="label">Memory</div><div class="value">${memPerc} — ${memUsed} / ${memTotal}</div></div>
          <div class="row"><div class="label">Disk (root)</div><div class="value">${disk}</div></div>
          <div class="row"><div class="label">Temp</div><div class="value">${temp}</div></div>
          <div class="row"><div class="label">Uptime</div><div class="value">${uptime}</div></div>
        </div>
      `;
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
