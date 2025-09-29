import { LitElement, html, nothing } from 'https://unpkg.com/lit@3.1.4/index.js?module';

/**
 * Compact status pill for a ROS topic subscription.
 *
 * @example
 * html`<pilot-topic-state .record=${record}></pilot-topic-state>`
 */
class PilotTopicState extends LitElement {
  static properties = {
    record: { type: Object },
  };

  constructor() {
    super();
    this.record = null;
  }

  createRenderRoot() {
    return this;
  }

  get status() {
    return this.record?.state ?? 'idle';
  }

  get isPaused() {
    return Boolean(this.record?.paused);
  }

  get errorMessage() {
    return typeof this.record?.error === 'string' && this.record.error.trim() ? this.record.error.trim() : null;
  }

  render() {
    return html`
      <div class="topic-state">
        <span class=${`state ${this.status}`}>${this.status}</span>
        ${this.isPaused ? html`<span class="paused">Paused</span>` : nothing}
        ${this.errorMessage ? html`<span class="error">${this.errorMessage}</span>` : nothing}
      </div>
    `;
  }
}

customElements.define('pilot-topic-state', PilotTopicState);
