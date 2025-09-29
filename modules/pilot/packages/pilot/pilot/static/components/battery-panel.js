import { LitElement, html } from 'https://unpkg.com/lit@3.1.4/index.js?module';

import { subscribeBattery, batteryLabel, batteryUnit, chargingStateLabel, updateBatteryMetric } from '../utils/battery.js';
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

  render() {
    const percent = Math.round(this.percent);
    const charge = this._metrics['/battery/charge'];
    const capacity = this._metrics['/battery/capacity'];
    const state = this._metrics['/battery/charging_state'];
    const stats = [
      {
        topic: '/battery/charging_state',
        label: batteryLabel('/battery/charging_state'),
        value: state?.label ?? chargingStateLabel(state),
      },
      {
        topic: '/battery/charge',
        label: batteryLabel('/battery/charge'),
        value: formatNumber(extractNumeric(charge, Number.NaN), batteryUnit('/battery/charge')),
      },
      {
        topic: '/battery/capacity',
        label: batteryLabel('/battery/capacity'),
        value: formatNumber(extractNumeric(capacity, Number.NaN), batteryUnit('/battery/capacity')),
      },
    ];
    return html`
      <div class="battery-panel" data-state=${this.record?.state ?? 'idle'}>
        <div class="battery-gauge">
          <div class="battery-fill" style=${`width: ${percent}%`}></div>
          <span class="battery-percent">${Number.isFinite(percent) ? `${percent}%` : '—'}</span>
        </div>
        <div class="battery-overview">
          <dl class="battery-stats">
            ${stats
              .filter((entry) => typeof entry.value === 'string' && entry.value.trim())
              .map(
                (entry) => html`
                  <div class="battery-stat" data-topic=${entry.topic}>
                    <dt>${entry.label}</dt>
                    <dd>${entry.value}</dd>
                  </div>
                `,
              )}
          </dl>
          <p class="battery-note">
            Detailed telemetry now lives with each battery topic — expand them for per-sensor views.
          </p>
        </div>
      </div>
    `;
  }
}

customElements.define('pilot-battery-panel', PilotBatteryPanel);
