import { LitElement, html, css } from 'https://unpkg.com/lit@3.1.4/index.js?module';
import { surfaceStyles } from '/components/pilot-style.js';
import { buildFeltIntentPayload } from './felt-dashboard.helpers.js';

function makeId(prefix) {
  return crypto.randomUUID ? crypto.randomUUID() : `${prefix}-${Date.now()}-${Math.random().toString(16).slice(2)}`;
}

/**
 * Dashboard for orchestrating Felt intent overrides.
 *
 * Custom events emitted for backend wiring:
 * - ``felt-intent-submit`` → `{ detail: FeltIntentPayload }`
 */
class FeltDashboard extends LitElement {
  static properties = {
    valence: { state: true },
    arousal: { state: true },
    stance: { state: true },
    context: { state: true },
    statusMessage: { state: true },
    statusTone: { state: true },
    intentLog: { state: true },
  };

  static styles = [
    surfaceStyles,
    css`
      form {
        display: grid;
        gap: 0.75rem;
      }

      .form-row {
        display: grid;
        gap: 0.5rem;
      }

      label {
        display: flex;
        flex-direction: column;
        gap: 0.35rem;
        font-size: 0.75rem;
        letter-spacing: 0.05em;
        text-transform: uppercase;
        color: var(--metric-label-color);
      }

      input[type='range'] {
        width: 100%;
      }

      textarea {
        font: inherit;
        padding: 0.6rem 0.75rem;
        border-radius: 0.5rem;
        border: 1px solid var(--control-surface-border);
        background: rgba(0, 0, 0, 0.3);
        color: var(--lcars-text);
        min-height: 100px;
        resize: vertical;
      }

      .intent-log {
        list-style: none;
        padding: 0;
        margin: 0;
        display: flex;
        flex-direction: column;
        gap: 0.5rem;
        max-height: 260px;
        overflow-y: auto;
      }

      .intent-entry {
        background: rgba(0, 0, 0, 0.35);
        border: 1px solid var(--control-surface-border);
        border-radius: 0.5rem;
        padding: 0.6rem;
        display: grid;
        gap: 0.35rem;
      }

      .intent-entry__meta {
        display: flex;
        gap: 0.5rem;
        font-size: 0.75rem;
        color: var(--lcars-muted);
      }
    `,
  ];

  constructor() {
    super();
    this.valence = 0;
    this.arousal = 0.2;
    this.stance = 0.5;
    this.context = '';
    this.statusMessage = 'Compose a feeling intent to steer downstream planners.';
    this.statusTone = 'info';
    this.intentLog = [];
  }

  render() {
    return html`
      <div class="surface-grid surface-grid--wide">
        <article class="surface-card surface-card--wide">
          <h3 class="surface-card__title">Feeling intent composer</h3>
          <p class="surface-status" data-variant="${this.statusTone}">${this.statusMessage}</p>
          <form @submit=${this.handleIntentSubmit}>
            <div class="form-row">
              <label>
                Valence (${this.valence.toFixed(2)})
                <input
                  type="range"
                  min="-1"
                  max="1"
                  step="0.05"
                  .value=${String(this.valence)}
                  @input=${(event) => (this.valence = Number(event.target.value))}
                />
              </label>
              <label>
                Arousal (${this.arousal.toFixed(2)})
                <input
                  type="range"
                  min="0"
                  max="1"
                  step="0.05"
                  .value=${String(this.arousal)}
                  @input=${(event) => (this.arousal = Number(event.target.value))}
                />
              </label>
              <label>
                Stance (${this.stance.toFixed(2)})
                <input
                  type="range"
                  min="0"
                  max="1"
                  step="0.05"
                  .value=${String(this.stance)}
                  @input=${(event) => (this.stance = Number(event.target.value))}
                />
              </label>
            </div>
            <label>
              Narrative context
              <textarea
                name="context"
                placeholder="Summarise the desired behaviour or motivation..."
                .value=${this.context}
                @input=${(event) => (this.context = event.target.value)}
              ></textarea>
            </label>
            <div class="surface-actions">
              <button type="submit" class="surface-button">Broadcast intent</button>
              <button type="button" class="surface-button surface-button--ghost" @click=${this.resetForm}>
                Reset composer
              </button>
            </div>
          </form>
        </article>

        <article class="surface-card surface-card--wide">
          <h3 class="surface-card__title">Intent log</h3>
          ${this.intentLog.length
            ? html`<ol class="intent-log">
                ${this.intentLog.map(
                  (entry) => html`<li class="intent-entry">
                    <div class="intent-entry__meta">
                      <span>${entry.timestamp}</span>
                      <span>V:${entry.valence.toFixed(2)}</span>
                      <span>A:${entry.arousal.toFixed(2)}</span>
                      <span>S:${entry.stance.toFixed(2)}</span>
                    </div>
                    <p class="intent-entry__context">${entry.context}</p>
                  </li>`
                )}
              </ol>`
            : html`<p class="surface-empty">No intents broadcast yet.</p>`}
          <div class="surface-actions">
            <button type="button" class="surface-button surface-button--ghost" @click=${this.simulatePulse}>
              Simulate inbound intent
            </button>
          </div>
        </article>
      </div>
    `;
  }

  handleIntentSubmit(event) {
    event.preventDefault();
    const payload = buildFeltIntentPayload({
      valence: this.valence,
      arousal: this.arousal,
      stance: this.stance,
      context: this.context,
    });
    if (!payload.ok) {
      this.statusMessage = payload.error;
      this.statusTone = 'error';
      return;
    }
    this.dispatchEvent(
      new CustomEvent('felt-intent-submit', {
        detail: payload.value,
        bubbles: true,
        composed: true,
      }),
    );
    this.statusMessage = 'Intent queued for broadcasting downstream.';
    this.statusTone = 'success';
    this.recordIntent(payload.value, 'pilot override');
  }

  resetForm() {
    this.valence = 0;
    this.arousal = 0.2;
    this.stance = 0.5;
    this.context = '';
    this.statusMessage = 'Composer reset to neutral baseline.';
    this.statusTone = 'info';
  }

  simulatePulse() {
    const entry = {
      valence: Math.random() * 2 - 1,
      arousal: Math.random(),
      stance: Math.random(),
      context: 'Simulated mission feedback pulse.',
    };
    this.recordIntent(entry, 'autonomy');
  }

  recordIntent(intent, source) {
    const logEntry = {
      id: makeId('felt'),
      timestamp: new Date().toLocaleTimeString(),
      valence: intent.valence,
      arousal: intent.arousal,
      stance: intent.stance,
      context: `[${source}] ${intent.context}`,
    };
    this.intentLog = [logEntry, ...this.intentLog].slice(0, 40);
  }
}

customElements.define('felt-dashboard', FeltDashboard);
