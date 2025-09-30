import { LitElement, html } from 'https://unpkg.com/lit@3.1.4/index.js?module';

import { updateBatteryMetric, batteryLabel } from '../utils/battery.js';
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
    const label = batteryLabel(this.topic?.topic ?? this.topic?.name ?? 'Battery metric');
    const formatted = Number.isFinite(this._value) ? this._value.toFixed(2) : '—';
    return html`
      <div class="battery-feed" data-state=${this.record?.state ?? 'idle'}>
        <span class="feed-label">${label}</span>
        <span class="feed-value">${formatted}</span>
        <span class="feed-status">Linked to battery panel</span>
      </div>
    `;
  }
}

customElements.define('pilot-battery-feed', PilotBatteryFeed);
