import { LitElement, html, css } from 'https://unpkg.com/lit@3.1.4/index.js?module';
import { surfaceStyles } from '/components/cockpit-style.js';
import {
  buildMemoryQueryPayload,
  buildMemoryStorePayload,
} from './memory-dashboard.helpers.js';

function makeId(prefix) {
  return crypto.randomUUID ? crypto.randomUUID() : `${prefix}-${Date.now()}-${Math.random().toString(16).slice(2)}`;
}

/**
 * Memory module dashboard for querying and seeding semantic memories.
 *
 * Events emitted:
 * - ``memory-query-request`` → `{ detail: MemoryQueryPayload }`
 * - ``memory-store-request`` → `{ detail: MemoryStorePayload }`
 */
class MemoryDashboard extends LitElement {
  static properties = {
    statusMessage: { state: true },
    statusTone: { state: true },
    queryText: { state: true },
    queryTopK: { state: true },
    queryFeedback: { state: true },
    queryResults: { state: true },
    storeTitle: { state: true },
    storeBody: { state: true },
    storeTags: { state: true },
    storeFeedback: { state: true },
  };

  static styles = [
    surfaceStyles,
    css`
      .result-entry {
        background: rgba(0, 0, 0, 0.35);
        border: 1px solid var(--control-surface-border);
        border-radius: 0.5rem;
        padding: 0.6rem;
        display: grid;
        gap: 0.35rem;
      }

      .result-entry__meta {
        display: flex;
        gap: 0.5rem;
        font-size: 0.75rem;
        color: var(--lcars-muted);
      }
    `,
  ];

  constructor() {
    super();
    this.statusMessage = 'Query the knowledge base or persist new insights.';
    this.statusTone = 'info';
    this.queryText = '';
    this.queryTopK = 5;
    this.queryFeedback = '';
    this.queryResults = [];
    this.storeTitle = '';
    this.storeBody = '';
    this.storeTags = '';
    this.storeFeedback = '';
  }

  render() {
    return html`
      <div class="surface-grid surface-grid--wide surface-grid--dense">
        <article class="surface-card surface-card--wide surface-card--compact">
          <h3 class="surface-card__title">Memory search</h3>
          <p class="surface-status" data-variant="${this.statusTone}">${this.statusMessage}</p>
          <form class="surface-form surface-form--compact" @submit=${this.handleQuerySubmit}>
            <label class="surface-field">
              <span class="surface-label">Query text</span>
              <textarea
                class="surface-textarea"
                name="query"
                placeholder="e.g. What did Pete observe during the afternoon patrol?"
                .value=${this.queryText}
                @input=${(event) => (this.queryText = event.target.value)}
              ></textarea>
            </label>
            <div class="surface-grid surface-grid--dense surface-grid--narrow">
              <label class="surface-field">
                <span class="surface-label">Results</span>
                <input
                  class="surface-input surface-input--small"
                  type="number"
                  min="1"
                  max="20"
                  .value=${String(this.queryTopK)}
                  @input=${(event) => (this.queryTopK = Number(event.target.value))}
                />
              </label>
            </div>
            ${this.queryFeedback
              ? html`<p class="surface-status" data-variant="error">${this.queryFeedback}</p>`
              : ''}
            <div class="surface-actions">
              <button type="submit" class="surface-button">Run query</button>
              <button type="button" class="surface-button surface-button--ghost" @click=${this.clearQuery}>
                Clear
              </button>
            </div>
          </form>
          ${this.queryResults.length
            ? html`<ol class="surface-list surface-list--scrollable">
                ${this.queryResults.map(
                  (entry) => html`<li class="result-entry">
                    <div class="result-entry__meta">
                      <span>${entry.timestamp}</span>
                      <span>${entry.score.toFixed(2)}</span>
                      ${entry.tags.length
                        ? html`<span>${entry.tags.map((tag) => `#${tag}`).join(' ')}</span>`
                        : ''}
                    </div>
                    <h4 class="result-entry__title">${entry.title}</h4>
                    <p class="result-entry__body">${entry.body}</p>
                  </li>`
                )}
              </ol>`
            : html`<p class="surface-empty">No query results yet.</p>`}
        </article>

        <article class="surface-card surface-card--wide surface-card--compact">
          <h3 class="surface-card__title">Store new memory</h3>
          ${this.storeFeedback
            ? html`<p class="surface-status" data-variant="error">${this.storeFeedback}</p>`
            : ''}
          <form class="surface-form surface-form--compact" @submit=${this.handleStoreSubmit}>
            <label class="surface-field">
              <span class="surface-label">Title</span>
              <input
                class="surface-input"
                type="text"
                name="title"
                .value=${this.storeTitle}
                @input=${(event) => (this.storeTitle = event.target.value)}
              />
            </label>
            <label class="surface-field">
              <span class="surface-label">Details</span>
              <textarea
                class="surface-textarea"
                name="body"
                .value=${this.storeBody}
                @input=${(event) => (this.storeBody = event.target.value)}
              ></textarea>
            </label>
            <label class="surface-field">
              <span class="surface-label">Tags</span>
              <input
                class="surface-input"
                type="text"
                name="tags"
                placeholder="comma,separated,tags"
                .value=${this.storeTags}
                @input=${(event) => (this.storeTags = event.target.value)}
              />
            </label>
            <div class="surface-actions">
              <button type="submit" class="surface-button">Persist memory</button>
              <button type="button" class="surface-button surface-button--ghost" @click=${this.clearStoreForm}>
                Clear form
              </button>
              <button type="button" class="surface-button surface-button--ghost" @click=${this.simulateIngest}>
                Simulate ingest
              </button>
            </div>
          </form>
        </article>
      </div>
    `;
  }

  handleQuerySubmit(event) {
    event.preventDefault();
    const payload = buildMemoryQueryPayload({ query: this.queryText, topK: this.queryTopK });
    if (!payload.ok) {
      this.queryFeedback = payload.error;
      return;
    }
    this.queryFeedback = '';
    this.dispatchEvent(
      new CustomEvent('memory-query-request', {
        detail: payload.value,
        bubbles: true,
        composed: true,
      }),
    );
    this.statusMessage = 'Query dispatched to the memory service.';
    this.statusTone = 'success';
    this.recordResult({
      title: 'Simulated recall',
      body: 'Pete greeted visitors near the entrance and recorded badge IDs.',
      score: Math.random(),
      tags: ['simulation'],
    });
  }

  handleStoreSubmit(event) {
    event.preventDefault();
    const payload = buildMemoryStorePayload({
      title: this.storeTitle,
      body: this.storeBody,
      tags: this.storeTags,
    });
    if (!payload.ok) {
      this.storeFeedback = payload.error;
      return;
    }
    this.storeFeedback = '';
    this.dispatchEvent(
      new CustomEvent('memory-store-request', {
        detail: payload.value,
        bubbles: true,
        composed: true,
      }),
    );
    this.statusMessage = 'Memory submitted for persistence.';
    this.statusTone = 'success';
    this.clearStoreForm();
  }

  clearQuery() {
    this.queryText = '';
    this.queryTopK = 5;
    this.queryFeedback = '';
  }

  clearStoreForm() {
    this.storeTitle = '';
    this.storeBody = '';
    this.storeTags = '';
  }

  simulateIngest() {
    this.recordResult({
      title: this.storeTitle || 'Synthetic event',
      body: this.storeBody || 'Generated sample memory entry.',
      score: 0.42,
      tags: this.storeTags ? this.storeTags.split(',').map((tag) => tag.trim()).filter(Boolean) : [],
    });
  }

  recordResult(result) {
    const entry = {
      id: makeId('memory'),
      timestamp: new Date().toLocaleTimeString(),
      title: result.title,
      body: result.body,
      score: result.score ?? 0,
      tags: Array.isArray(result.tags) ? result.tags : [],
    };
    this.queryResults = [entry, ...this.queryResults].slice(0, 30);
  }
}

customElements.define('memory-dashboard', MemoryDashboard);
