import { LitElement, html, nothing } from 'https://unpkg.com/lit@3.1.4/index.js?module';

// regimes grouping removed
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
    this._topicObserver = null;
  }

  createRenderRoot() {
    return this;
  }

  get moduleRegimes() {
    return [];
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
    // Clear any manual-stop flag when a user explicitly starts a topic
    try {
      const id = encodeURIComponent(topicIdentifier(topic));
      const el = this.querySelector(`.topic-card[data-topic="${id}"]`);
      if (el) {
        delete el.dataset.manualStopped;
      }
    } catch (_e) {
      // ignore
    }

    this.dispatchEvent(
      new CustomEvent('start-topic', {
        detail: { module: this.module?.name, topic },
        bubbles: true,
        composed: true,
      }),
    );
  }

  stopTopic(topic) {
    // Mark this control as manually stopped so the auto-subscribe logic
    // doesn't immediately reconnect when the user intentionally disconnects.
    try {
      const id = encodeURIComponent(topicIdentifier(topic));
      const el = this.querySelector(`.topic-card[data-topic="${id}"]`);
      if (el) {
        el.dataset.manualStopped = 'true';
      }
    } catch (_e) {
      // ignore
    }

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
    return nothing;
  }

  renderCommands(scope, commands) {
    if (!commands?.length) {
      return html`<div class="button-row empty">No commands</div>`;
    }
    return html`
      <div class="button-row">
        ${commands.map(
      (name) => html`
            <button
              type="button"
              class="control-button"
              data-variant="ghost"
              @click=${() => this.command(scope, name)}
            >
              ${name}
            </button>
          `,
    )}
      </div>
    `;
  }

  renderPilotLink() {
    if (!this.module?.has_pilot) {
      return nothing;
    }
    const label = this.module.display_name || this.module.name;
    return html`
      <div class="command-set command-set--pilot">
        <h3>Dashboard</h3>
        <div class="button-row">
          <a
            class="control-button"
            data-variant="accent"
            href=${`/modules/${this.module.name}/`}
            target="_blank"
            rel="noopener"
          >
            Open ${label}
          </a>
        </div>
      </div>
    `;
  }

  renderTopic(topic) {
    const record = this.topicRecord(topic);
    const identifier = topicIdentifier(topic);
    return html`
      <article class="topic-card" data-topic=${encodeURIComponent(identifier)}>
        <div class="topic-header">
          <div>
            <h4>${identifier}</h4>
            <small>${topic.type}</small>
          </div>
          <div class="topic-actions">
            ${record
        ? html`
                  <button
                    type="button"
                    class="control-button"
                    data-variant="critical"
                    @click=${() => this.stopTopic(topic)}
                  >
                    Disconnect
                  </button>
                  <button
                    type="button"
                    class="control-button"
                    data-variant=${record.paused ? 'accent' : 'ghost'}
                    @click=${() => this.togglePause(topic, !record.paused)}
                  >
                    ${record.paused ? 'Resume' : 'Pause'}
                  </button>
                `
        : html`<button
                type="button"
                class="control-button"
                data-variant="accent"
                @click=${() => this.startTopic(topic)}
              >
                Connect
              </button>`}
          </div>
        </div>
        <pilot-topic-widget .record=${record} .topic=${topic}></pilot-topic-widget>
      </article>
    `;
  }

  firstUpdated() {
    // Create a gentle IntersectionObserver to auto-subscribe when a topic
    // control is scrolled into view.
    try {
      if (this._topicObserver) return;
      this._topicObserver = new IntersectionObserver(
        (entries) => {
          for (const entry of entries) {
            const el = entry.target;
            const encoded = el?.dataset?.topic;
            if (!encoded) continue;
            const topicName = decodeURIComponent(encoded);
            // Only act when the control is meaningfully visible.
            if (entry.isIntersecting && entry.intersectionRatio >= 0.25) {
              // Find the topic object by its identifier.
              const topicObj = (this.module?.topics || []).find((t) => topicIdentifier(t) === topicName);
              if (!topicObj) continue;
              const record = this.topicRecord(topicObj);
              const manualStopped = el.dataset.manualStopped === 'true';
              const access = topicObj.access || 'ro';
              // Gentle policy: only auto-subscribe read-only topics, skip write-only.
              if (!record && !manualStopped && access !== 'wo') {
                this.startTopic(topicObj);
                el.dataset.autoSubscribed = 'true';
              }
            }
          }
        },
        { root: null, rootMargin: '0px 0px 200px 0px', threshold: [0, 0.25] },
      );
    } catch (_e) {
      console.warn('Failed to create topic observer', _e);
    }
    // Observe any initially rendered topic cards.
    this.observeTopics();
  }

  updated() {
    // Re-attach observer to any newly rendered topic cards.
    this.observeTopics();
  }

  observeTopics() {
    if (!this._topicObserver) return;
    try {
      const cards = this.querySelectorAll('.topic-card');
      for (const card of cards) {
        if (card.__observed) continue;
        this._topicObserver.observe(card);
        card.__observed = true;
      }
    } catch (e) {
      // swallow errors to avoid breaking rendering
    }
  }

  disconnectedCallback() {
    // Clean up the observer when the element is removed from the DOM.
    try {
      if (this._topicObserver) {
        this._topicObserver.disconnect();
        this._topicObserver = null;
      }
    } catch (e) {
      // ignore
    }
    super.disconnectedCallback();
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
          <div>
            <h2>${this.module.display_name || this.module.name}</h2>
            ${this.module.description ? html`<p>${this.module.description}</p>` : nothing}
            ${this.renderRegimeTags()}
          </div>
          <div class="command-groups">
            ${this.renderPilotLink()}
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
