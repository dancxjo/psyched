import { LitElement, html } from 'https://unpkg.com/lit@3.1.4/index.js?module';

import { extractNumeric } from '/utils/metrics.js';

function inferRange(topicName) {
  if (!topicName) {
    return { min: 0, max: 100 };
  }
  if (topicName.includes('ratio')) {
    return { min: 0, max: 1 };
  }
  if (topicName.includes('ms')) {
    return { min: 0, max: 5000 };
  }
  if (topicName.includes('temperature')) {
    return { min: -10, max: 80 };
  }
  if (topicName.includes('voltage')) {
    return { min: 0, max: 20 };
  }
  if (topicName.includes('current')) {
    return { min: -5, max: 5 };
  }
  return { min: 0, max: 100 };
}

/**
 * Render a compact numeric gauge with a progress indicator.
 */
class CockpitValueGauge extends LitElement {
  static properties = {
    record: { type: Object },
    topic: { type: Object },
    _value: { state: true },
    _range: { state: true },
  };

  constructor() {
    super();
    this.record = null;
    this.topic = null;
    this._value = 0;
    this._range = { min: 0, max: 100 };
  }

  createRenderRoot() {
    return this;
  }

  updated(changed) {
    if (changed.has('topic')) {
      const topicName = this.topic?.topic ?? this.topic?.name ?? '';
      this._range = inferRange(String(topicName));
    }
    if (changed.has('record')) {
      const next = extractNumeric(this.record?.last, this._value);
      this._value = Number.isFinite(next) ? next : this._value;
    }
  }

  get percent() {
    const { min, max } = this._range;
    const span = max - min || 1;
    const clamped = Math.max(min, Math.min(max, this._value));
    return ((clamped - min) / span) * 100;
  }

  render() {
    if (!this.record) {
      return html`<div class="gauge inactive">Awaiting data…</div>`;
    }
    const { min, max } = this._range;
    const displayValue = Number.isFinite(this._value) ? this._value.toFixed(2) : '—';
    const percentage = Math.round(this.percent);
    return html`
      <div class="gauge" role="img" aria-label="Numeric gauge">
        <div class="gauge-track">
          <div class="gauge-fill" style=${`width: ${percentage}%`}></div>
        </div>
        <div class="gauge-meta">
          <span class="gauge-value">${displayValue}</span>
          <span class="gauge-range">${min} – ${max}</span>
        </div>
      </div>
    `;
  }
}

customElements.define('cockpit-value-gauge', CockpitValueGauge);
