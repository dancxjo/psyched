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
    gap: 1rem;
  }

  .surface-grid--dense {
    gap: 0.75rem;
  }

  .surface-grid--wide {
    grid-template-columns: repeat(auto-fit, minmax(260px, 1fr));
  }

  .surface-grid--medium {
    grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
  }

  .surface-grid--narrow {
    grid-template-columns: repeat(auto-fit, minmax(180px, 1fr));
  }

  .surface-grid--stack {
    grid-template-columns: minmax(0, 1fr);
  }

  .surface-card {
    background: var(--control-surface-bg);
    border: 1px solid var(--control-surface-border);
    border-radius: var(--control-surface-radius);
    padding: var(--control-surface-padding);
    box-shadow: var(--control-surface-shadow);
    display: flex;
    flex-direction: column;
    gap: 0.75rem;
    backdrop-filter: blur(6px);
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

  .surface-action {
    background: rgba(88, 178, 220, 0.2);
    border: 1px solid var(--control-surface-border);
    border-radius: 0.5rem;
    padding: 0.6rem 0.85rem;
    text-transform: uppercase;
    letter-spacing: 0.05em;
    font-size: 0.7rem;
    font-weight: 600;
    color: var(--lcars-text);
    cursor: pointer;
    transition: background 150ms ease, transform 150ms ease;
  }

  .surface-action:hover,
  .surface-action:focus {
    background: rgba(88, 178, 220, 0.35);
    transform: translateY(-1px);
  }

  .surface-panel {
    background: rgba(0, 0, 0, 0.3);
    border: 1px solid var(--control-surface-border);
    border-radius: 0.5rem;
    padding: 0.5rem;
  }

  .surface-mono {
    font-family: var(--metric-value-font);
  }

  .surface-muted {
    color: var(--lcars-muted);
  }
`;
