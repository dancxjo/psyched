import { LitElement, html, nothing } from 'https://unpkg.com/lit@3.1.4/index.js?module';

import { updateBatteryMetric, batteryLabel, batteryUnit, chargingStateLabel } from '../utils/battery.js';
import { extractNumeric } from '../utils/metrics.js';

/**
 * Lightweight component that forwards a battery metric into the shared store.
 */
class PilotBatteryFeed extends LitElement {
  static properties = {
    record: { type: Object },
    topic: { type: Object },
    _value: { state: true },
  };

  constructor() {
    super();
    this.record = null;
    this.topic = null;
    this._value = Number.NaN;
  }

  createRenderRoot() {
    return this;
  }

  updated(changed) {
    if (changed.has('record') && this.record) {
      const topicName = this.topic?.topic ?? this.topic?.name;
      if (topicName) {
        const latest = this.record?.last ?? this.record?.messages?.[0];
        this._value = extractNumeric(latest, this._value);
        updateBatteryMetric(topicName, latest);
      }
    }
  }

  render() {
    const topicName = this.topic?.topic ?? this.topic?.name ?? '';
    const label = topicName ? batteryLabel(topicName) : 'Battery metric';
    let valueText = '—';
    let detail = nothing;

    if (topicName === '/battery/charging_state') {
      const numeric = extractNumeric(this.record?.last ?? this.record?.messages?.[0], Number.NaN);
      const stateLabel = chargingStateLabel(numeric);
      valueText = stateLabel;
      if (Number.isFinite(numeric)) {
        detail = html`<span class="feed-status">Code ${numeric}</span>`;
      }
    } else {
      const unit = batteryUnit(topicName);
      if (Number.isFinite(this._value)) {
        const precise = Math.abs(this._value) >= 10 ? this._value.toFixed(1) : this._value.toFixed(2);
        valueText = unit ? `${precise} ${unit}` : precise;
      }
    }

    return html`
      <div class="battery-feed" data-state=${this.record?.state ?? 'idle'}>
        <span class="feed-label">${label}</span>
        <span class="feed-value">${valueText}</span>
        ${detail}
      </div>
    `;
  }
}

customElements.define('pilot-battery-feed', PilotBatteryFeed);
