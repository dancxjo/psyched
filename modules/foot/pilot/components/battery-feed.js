import { LitElement, html } from 'https://unpkg.com/lit@3.1.4/index.js?module';

import { batteryLabel, batteryUnit, formatBatteryValue } from '../utils/battery.js';

/**
 * Displays an individual battery telemetry feed with contextual units.
 */
class PilotBatteryFeed extends LitElement {
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
    return this.topic?.topic ?? this.topic?.name ?? '';
  }

  get latestPayload() {
    return this.record?.last ?? this.record?.messages?.[0];
  }

  get formattedValue() {
    return formatBatteryValue(this.topicName, this.latestPayload);
  }

  render() {
    const label = batteryLabel(this.topicName || 'battery_metric');
    const unit = batteryUnit(this.topicName);
    const topicPath = this.topicName || '—';
    const statusParts = [topicPath ? `Topic: ${topicPath}` : null, unit ? `Unit: ${unit}` : null].filter(Boolean);
    const status = statusParts.length ? statusParts.join(' • ') : 'Awaiting telemetry';
    return html`
      <div class="battery-feed" data-state=${this.record?.state ?? 'idle'}>
        <span class="label feed-label">${label}</span>
        <span class="feed-value">${this.formattedValue}</span>
        <span class="feed-status">${status}</span>
      </div>
    `;
  }
}

customElements.define('pilot-battery-feed', PilotBatteryFeed);
