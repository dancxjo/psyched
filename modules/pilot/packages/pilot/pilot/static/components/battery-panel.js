import { LitElement, html } from 'https://unpkg.com/lit@3.1.4/index.js?module';

import { subscribeBattery, batteryLabel, updateBatteryMetric } from '../utils/battery.js';
import { extractNumeric } from '../utils/metrics.js';

function formatNumber(value, unit = '') {
  if (!Number.isFinite(value)) {
    return '—';
  }
  const fixed = Math.abs(value) >= 10 ? value.toFixed(1) : value.toFixed(2);
  return unit ? `${fixed} ${unit}` : fixed;
}

/**
 * Aggregated battery dashboard combining multiple telemetry feeds.
 */
class PilotBatteryPanel extends LitElement {
  static properties = {
    record: { type: Object },
    topic: { type: Object },
    _metrics: { state: true },
  };

  constructor() {
    super();
    this.record = null;
    this.topic = null;
    this._metrics = {};
    this._unsubscribe = null;
  }

  createRenderRoot() {
    return this;
  }

  connectedCallback() {
    super.connectedCallback();
    this._unsubscribe = subscribeBattery((metrics) => {
      this._metrics = metrics;
      this.requestUpdate();
    });
  }

  disconnectedCallback() {
    super.disconnectedCallback();
    if (typeof this._unsubscribe === 'function') {
      this._unsubscribe();
    }
    this._unsubscribe = null;
  }

  updated(changed) {
    if (changed.has('record') && this.record) {
      const topicName = this.topic?.topic ?? this.topic?.name;
      if (topicName) {
        const value = this.record?.last ?? this.record?.messages?.[0];
        updateBatteryMetric(topicName, value);
      }
    }
  }

  get percent() {
    const raw = this._metrics['/battery/charge_ratio'];
    const numeric = extractNumeric(raw, Number.NaN);
    if (!Number.isFinite(numeric)) {
      return 0;
    }
    return Math.max(0, Math.min(1, numeric)) * 100;
  }

  renderMetric(topic, unit = '') {
    const value = this._metrics[topic];
    if (topic === '/battery/charging_state') {
      const label = value?.label ?? 'Unknown';
      return html`<li><span class="label">${batteryLabel(topic)}</span><span>${label}</span></li>`;
    }
    const numeric = extractNumeric(value, Number.NaN);
    return html`<li>
      <span class="label">${batteryLabel(topic)}</span>
      <span>${formatNumber(numeric, unit)}</span>
    </li>`;
  }

  render() {
    const percent = Math.round(this.percent);
    return html`
      <div class="battery-panel" data-state=${this.record?.state ?? 'idle'}>
        <div class="battery-gauge">
          <div class="battery-fill" style=${`width: ${percent}%`}></div>
          <span class="battery-percent">${Number.isFinite(percent) ? `${percent}%` : '—'}</span>
        </div>
        <ul class="battery-metrics">
          ${this.renderMetric('/battery/voltage', 'V')}
          ${this.renderMetric('/battery/current', 'A')}
          ${this.renderMetric('/battery/charge', 'mAh')}
          ${this.renderMetric('/battery/capacity', 'mAh')}
          ${this.renderMetric('/battery/temperature', '°C')}
          ${this.renderMetric('/battery/charging_state')}
        </ul>
      </div>
    `;
  }
}

customElements.define('pilot-battery-panel', PilotBatteryPanel);
