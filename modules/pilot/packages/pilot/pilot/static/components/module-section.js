import { LitElement, html, nothing } from 'https://unpkg.com/lit@3.1.4/index.js?module';

import { formatRegimeName, normalizeRegimes } from '../utils/regimes.js';
import { topicKey, topicIdentifier } from '../utils/topics.js';
import './topic-widget.js';

/**
 * Presents a single module card, including command buttons and topic widgets.
 */
class PilotModuleSection extends LitElement {
  static properties = {
    module: { type: Object },
    activeRecords: { type: Object },
  };

  constructor() {
    super();
    this.module = null;
    this.activeRecords = new Map();
  }

  createRenderRoot() {
    return this;
  }

  get moduleRegimes() {
    return normalizeRegimes(this.module?.regimes ?? []);
  }

  command(scope, command) {
    this.dispatchEvent(
      new CustomEvent('run-command', {
        detail: { scope, command },
        bubbles: true,
        composed: true,
      }),
    );
  }

  startTopic(topic) {
    this.dispatchEvent(
      new CustomEvent('start-topic', {
        detail: { module: this.module?.name, topic },
        bubbles: true,
        composed: true,
      }),
    );
  }

  stopTopic(topic) {
    this.dispatchEvent(
      new CustomEvent('stop-topic', {
        detail: { module: this.module?.name, topic },
        bubbles: true,
        composed: true,
      }),
    );
  }

  togglePause(topic, paused) {
    this.dispatchEvent(
      new CustomEvent('pause-topic', {
        detail: { module: this.module?.name, topic, paused },
        bubbles: true,
        composed: true,
      }),
    );
  }

  topicRecord(topic) {
    const records = this.activeRecords instanceof Map ? this.activeRecords : new Map(Object.entries(this.activeRecords || {}));
    return records.get(topicKey(this.module?.name, topic));
  }

  renderRegimeTags() {
    if (!this.moduleRegimes.length) {
      return nothing;
    }
    return html`
      <ul class="regime-tags">
        ${this.moduleRegimes.map((regime) => html`<li>${formatRegimeName(regime)}</li>`)}
      </ul>
    `;
  }

  renderCommands(scope, commands) {
    if (!commands?.length) {
      return html`<div class="button-row empty">No commands</div>`;
    }
    return html`
      <div class="button-row">
        ${commands.map(
          (name) => html`
            <button type="button" @click=${() => this.command(scope, name)}>${name}</button>
          `,
        )}
      </div>
    `;
  }

  renderTopic(topic) {
    const record = this.topicRecord(topic);
    const identifier = topicIdentifier(topic);
    return html`
      <article class="topic-card">
        <div class="topic-header">
          <div>
            <h4>${identifier}</h4>
            <small>${topic.type}</small>
          </div>
          <div class="topic-actions">
            ${record
              ? html`
                  <button type="button" @click=${() => this.stopTopic(topic)}>Disconnect</button>
                  <button type="button" @click=${() => this.togglePause(topic, !record.paused)}>
                    ${record.paused ? 'Resume' : 'Pause'}
                  </button>
                `
              : html`<button type="button" @click=${() => this.startTopic(topic)}>Connect</button>`}
          </div>
        </div>
        <pilot-topic-widget .record=${record} .topic=${topic}></pilot-topic-widget>
      </article>
    `;
  }

  renderTopics() {
    if (!this.module?.topics?.length) {
      return html`<p class="empty-topics">No topics declared.</p>`;
    }
    return html`
      <div class="topics-grid">
        ${this.module.topics.map((topic) => (topic ? this.renderTopic(topic) : nothing))}
      </div>
    `;
  }

  render() {
    if (!this.module) {
      return nothing;
    }
    return html`
      <section class="module-section" id=${`module-${this.module.name}`}>
        <header class="module-header">
          <h2>${this.module.display_name || this.module.name}</h2>
          ${this.module.description ? html`<p>${this.module.description}</p>` : nothing}
          ${this.renderRegimeTags()}
          <div class="command-groups">
            <div class="command-set">
              <h3>Module Commands</h3>
              ${this.renderCommands('mod', this.module.commands?.mod ?? [])}
            </div>
            <div class="command-set">
              <h3>System Commands</h3>
              ${this.renderCommands('sys', this.module.commands?.system ?? [])}
            </div>
          </div>
        </header>
        ${this.renderTopics()}
      </section>
    `;
  }
}

customElements.define('pilot-module-section', PilotModuleSection);
