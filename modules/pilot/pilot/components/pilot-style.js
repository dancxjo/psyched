import { css } from 'https://unpkg.com/lit@3.1.4/index.js?module';

/**
 * Shared surface styling primitives consumed by module dashboards.
 *
 * The helpers intentionally avoid layout-specific declarations so
 * individual modules can remain compact while reusing the same
 * typography and control affordances.
 */
export const surfaceStyles = css`
  :host {
    display: block;
    color: var(--lcars-text);
  }

  .surface-grid {
    display: grid;
    gap: 0.85rem;
  }

  .surface-grid--dense {
    gap: 0.6rem;
  }

  .surface-grid--wide {
    grid-template-columns: repeat(auto-fit, minmax(240px, 1fr));
  }

  .surface-grid--medium {
    grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
  }

  .surface-grid--narrow {
    grid-template-columns: repeat(auto-fit, minmax(160px, 1fr));
  }

  .surface-grid--stack {
    grid-template-columns: minmax(0, 1fr);
  }

  .surface-card {
    background: var(--control-surface-bg);
    border: 1px solid var(--control-surface-border);
    border-radius: var(--control-surface-radius);
    padding: calc(var(--control-surface-padding) - 0.25rem);
    box-shadow: var(--control-surface-shadow);
    display: flex;
    flex-direction: column;
    gap: 0.75rem;
    backdrop-filter: blur(6px);
  }

  .surface-card--compact {
    padding: calc(var(--control-surface-padding) - 0.35rem);
    gap: 0.65rem;
  }

  .surface-card__title {
    margin: 0;
    font-size: 0.9rem;
    letter-spacing: 0.08em;
    text-transform: uppercase;
    color: var(--metric-title-color);
  }

  .surface-card__subtitle {
    margin: 0;
    font-size: 0.8rem;
    color: var(--lcars-muted);
  }

  .surface-status {
    font-size: 0.8rem;
    color: var(--lcars-muted);
  }

  .surface-status[data-variant='success'] {
    color: var(--lcars-success);
  }

  .surface-status[data-variant='warning'] {
    color: var(--lcars-warning);
  }

  .surface-status[data-variant='error'] {
    color: var(--lcars-error);
  }

  .surface-chip {
    display: inline-flex;
    align-items: center;
    gap: 0.5rem;
    padding: 0.35rem 0.75rem;
    border-radius: 999px;
    font-size: 0.75rem;
    letter-spacing: 0.05em;
    text-transform: uppercase;
    background: rgba(88, 178, 220, 0.18);
    color: var(--lcars-text);
    align-self: flex-start;
  }

  .surface-chip[data-variant='success'] {
    background: rgba(92, 209, 132, 0.25);
  }

  .surface-chip[data-variant='critical'] {
    background: rgba(255, 111, 97, 0.25);
    color: var(--lcars-error);
  }

  .surface-chip[data-variant='warning'] {
    background: rgba(255, 192, 76, 0.25);
    color: var(--lcars-warning);
  }

  .surface-metric {
    display: flex;
    flex-direction: column;
    gap: 0.25rem;
  }

  .surface-metric--inline {
    flex-direction: row;
    align-items: baseline;
    justify-content: space-between;
    gap: 0.75rem;
  }

  .surface-metric__label {
    font-size: 0.75rem;
    letter-spacing: 0.08em;
    text-transform: uppercase;
    color: var(--metric-label-color);
  }

  .surface-metric__value {
    font-family: var(--metric-value-font);
    font-weight: 600;
    color: var(--lcars-text);
  }

  .surface-metric__value--large {
    font-size: 1.4rem;
  }

  .surface-form {
    display: grid;
    gap: 0.75rem;
  }

  .surface-form--compact {
    gap: 0.5rem;
  }

  .surface-form label,
  .surface-card form label {
    display: flex;
    flex-direction: column;
    gap: 0.35rem;
    font-size: 0.75rem;
    letter-spacing: 0.06em;
    text-transform: uppercase;
    color: var(--metric-label-color);
  }

  .surface-form label > input,
  .surface-form label > select,
  .surface-form label > textarea,
  .surface-card form label > input,
  .surface-card form label > select,
  .surface-card form label > textarea {
    font: inherit;
    padding: 0.55rem 0.75rem;
    border-radius: 0.5rem;
    border: 1px solid var(--control-surface-border);
    background: rgba(0, 0, 0, 0.28);
    color: var(--lcars-text);
    font-family: var(--metric-value-font);
    box-sizing: border-box;
  }

  .surface-form label > textarea,
  .surface-card form label > textarea {
    resize: vertical;
    min-height: 80px;
  }

  .surface-form label > select,
  .surface-card form label > select {
    appearance: none;
    background-image: linear-gradient(45deg, transparent 50%, rgba(255, 255, 255, 0.45) 50%),
      linear-gradient(135deg, rgba(255, 255, 255, 0.45) 50%, transparent 50%),
      linear-gradient(to right, transparent, transparent);
    background-position: calc(100% - 18px) calc(50% - 4px), calc(100% - 13px) calc(50% - 4px), calc(100% - 2.1rem) 0.45rem;
    background-size: 6px 6px, 6px 6px, 1px 70%;
    background-repeat: no-repeat;
  }

  .surface-card form {
    display: grid;
    gap: 0.65rem;
  }

  .surface-fieldset {
    margin: 0;
    padding: 0;
    border: none;
    display: grid;
    gap: 0.6rem;
  }

  .surface-fieldset--inline {
    gap: 0.5rem;
  }

  .surface-legend {
    font-size: 0.75rem;
    letter-spacing: 0.08em;
    text-transform: uppercase;
    color: var(--metric-label-color);
  }

  .surface-field {
    display: grid;
    gap: 0.35rem;
  }

  .surface-field--inline {
    display: flex;
    align-items: center;
    gap: 0.75rem;
  }

  .surface-label {
    font-size: 0.75rem;
    letter-spacing: 0.08em;
    text-transform: uppercase;
    color: var(--metric-label-color);
  }

  .surface-input,
  .surface-select,
  .surface-textarea,
  .surface-input[type='number'],
  .surface-input[type='text'],
  .surface-input[type='password'],
  .surface-input[type='search'] {
    font: inherit;
    padding: 0.55rem 0.75rem;
    border-radius: 0.5rem;
    border: 1px solid var(--control-surface-border);
    background: rgba(0, 0, 0, 0.28);
    color: var(--lcars-text);
    font-family: var(--metric-value-font);
    box-sizing: border-box;
  }

  .surface-input--small,
  .surface-select--small {
    padding: 0.4rem 0.6rem;
    font-size: 0.8rem;
  }

  .surface-textarea {
    resize: vertical;
    min-height: 80px;
  }

  .surface-select {
    appearance: none;
    background-image: linear-gradient(45deg, transparent 50%, rgba(255, 255, 255, 0.4) 50%),
      linear-gradient(135deg, rgba(255, 255, 255, 0.4) 50%, transparent 50%),
      linear-gradient(to right, transparent, transparent);
    background-position: calc(100% - 20px) calc(50% - 4px), calc(100% - 15px) calc(50% - 4px), calc(100% - 2.2rem) 0.45rem;
    background-size: 6px 6px, 6px 6px, 1px 70%;
    background-repeat: no-repeat;
  }

  .surface-select:focus {
    outline: none;
    border-color: rgba(88, 178, 220, 0.8);
    box-shadow: 0 0 0 2px rgba(88, 178, 220, 0.25);
  }

  .surface-input:focus,
  .surface-textarea:focus {
    outline: none;
    border-color: rgba(88, 178, 220, 0.7);
    box-shadow: 0 0 0 2px rgba(88, 178, 220, 0.2);
  }

  .surface-input[disabled],
  .surface-textarea[disabled],
  .surface-select[disabled] {
    opacity: 0.5;
    cursor: not-allowed;
  }

  .surface-log {
    list-style: none;
    margin: 0;
    padding: 0;
    display: flex;
    flex-direction: column;
    gap: 0.5rem;
    max-height: 260px;
    overflow-y: auto;
  }

  .surface-log__entry {
    background: rgba(0, 0, 0, 0.3);
    border: 1px solid var(--control-surface-border);
    border-radius: 0.5rem;
    padding: 0.6rem;
    display: flex;
    flex-direction: column;
    gap: 0.35rem;
    font-size: 0.85rem;
  }

  .surface-actions {
    display: flex;
    flex-wrap: wrap;
    gap: 0.5rem;
  }

  .surface-actions--grid {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(100px, 1fr));
    gap: 0.5rem;
  }

  .surface-button {
    display: inline-flex;
    align-items: center;
    justify-content: center;
    gap: 0.35rem;
    padding: 0.45rem 0.85rem;
    border-radius: 999px;
    border: 1px solid rgba(255, 255, 255, 0.15);
    background: rgba(88, 178, 220, 0.25);
    color: var(--lcars-text);
    text-transform: uppercase;
    letter-spacing: 0.06em;
    font-size: 0.7rem;
    font-weight: 600;
    cursor: pointer;
    transition: background 150ms ease, transform 150ms ease, box-shadow 150ms ease;
  }

  .surface-button:hover,
  .surface-button:focus {
    outline: none;
    background: rgba(88, 178, 220, 0.4);
    transform: translateY(-1px);
    box-shadow: 0 4px 10px rgba(0, 0, 0, 0.25);
  }

  .surface-button[disabled] {
    opacity: 0.45;
    cursor: not-allowed;
    transform: none;
  }

  .surface-button--ghost {
    background: rgba(255, 255, 255, 0.08);
    color: var(--lcars-muted);
  }

  .surface-button--critical {
    background: rgba(255, 111, 97, 0.25);
    color: var(--lcars-error);
  }

  .surface-button--success {
    background: rgba(92, 209, 132, 0.28);
    color: var(--lcars-success);
  }

  .surface-action {
    background: rgba(88, 178, 220, 0.25);
    border: 1px solid rgba(255, 255, 255, 0.15);
    border-radius: 999px;
    padding: 0.45rem 0.85rem;
    text-transform: uppercase;
    letter-spacing: 0.06em;
    font-size: 0.7rem;
    font-weight: 600;
    color: var(--lcars-text);
    cursor: pointer;
    transition: background 150ms ease, transform 150ms ease, box-shadow 150ms ease;
    display: inline-flex;
    align-items: center;
    justify-content: center;
    gap: 0.35rem;
  }

  .surface-action:hover,
  .surface-action:focus {
    background: rgba(88, 178, 220, 0.4);
    transform: translateY(-1px);
    box-shadow: 0 4px 10px rgba(0, 0, 0, 0.25);
  }

  .surface-action[disabled] {
    opacity: 0.45;
    cursor: not-allowed;
    transform: none;
    box-shadow: none;
  }

  .surface-panel {
    background: rgba(0, 0, 0, 0.3);
    border: 1px solid var(--control-surface-border);
    border-radius: 0.5rem;
    padding: 0.5rem;
  }

  .surface-list {
    list-style: none;
    margin: 0;
    padding: 0;
    display: flex;
    flex-direction: column;
    gap: 0.5rem;
  }

  .surface-list--scrollable {
    max-height: 260px;
    overflow-y: auto;
  }

  .surface-empty {
    margin: 0;
    text-align: center;
    font-size: 0.85rem;
    font-style: italic;
    color: var(--lcars-muted);
  }

  .surface-note {
    font-size: 0.75rem;
    color: var(--lcars-muted);
  }

  .surface-tag {
    display: inline-flex;
    align-items: center;
    padding: 0.2rem 0.6rem;
    border-radius: 999px;
    background: rgba(255, 255, 255, 0.08);
    font-size: 0.7rem;
    letter-spacing: 0.05em;
    text-transform: uppercase;
  }

  .surface-pill {
    display: inline-flex;
    align-items: center;
    gap: 0.35rem;
    padding: 0.3rem 0.65rem;
    border-radius: 999px;
    background: rgba(255, 255, 255, 0.1);
    font-size: 0.7rem;
    text-transform: uppercase;
    letter-spacing: 0.06em;
  }

  .surface-pill[data-variant='success'] {
    background: rgba(92, 209, 132, 0.25);
    color: var(--lcars-success);
  }

  .surface-pill[data-variant='muted'] {
    color: var(--lcars-muted);
  }

  .surface-mono {
    font-family: var(--metric-value-font);
  }

  .surface-muted {
    color: var(--lcars-muted);
  }
`;
