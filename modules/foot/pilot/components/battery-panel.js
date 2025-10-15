import { LitElement, html } from 'https://unpkg.com/lit@3.1.4/index.js?module';

import { batteryLabel, formatBatteryValue, batteryMetadata } from '../utils/battery.js';
import { extractNumeric } from '/components/utils/metrics.js';

/**
 * Presents the aggregated battery charge level for the active platform.
 */
class PilotBatteryPanel extends LitElement {
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

  get topicName() {
    return this.topic?.topic ?? this.topic?.name ?? '/battery/charge_ratio';
  }

  get latestPayload() {
    return this.record?.last ?? this.record?.messages?.[0];
  }

  get percent() {
    const metadata = batteryMetadata(this.topicName);
    const numeric = extractNumeric(this.latestPayload, Number.NaN);
    if (!Number.isFinite(numeric)) {
      return Number.NaN;
    }
    const scale = metadata.scale ?? 1;
    const scaled = scale === 100 && numeric > 1 ? numeric : numeric * scale;
    return Math.max(0, Math.min(100, scaled));
  }

  get formattedCharge() {
    return formatBatteryValue(this.topicName, this.latestPayload);
  }

  render() {
    const percent = this.percent;
    const chargeLabel = batteryLabel(this.topicName);
    return html`
      <div class="battery-panel" data-state=${this.record?.state ?? 'idle'}>
        <div class="battery-gauge">
          <div class="battery-fill" style=${Number.isFinite(percent) ? `width: ${percent}%` : 'width: 0%'}></div>
          <span class="battery-percent">${Number.isFinite(percent) ? `${Math.round(percent)}%` : '—'}</span>
        </div>
        <ul class="battery-metrics">
          <li>
            <span class="label">${chargeLabel}</span>
            <span>${this.formattedCharge}</span>
          </li>
        </ul>
      </div>
    `;
  }
}

customElements.define('pilot-battery-panel', PilotBatteryPanel);
