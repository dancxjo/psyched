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

  .surface-grid--Stack {
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

  .surface-card--collapsible {
    position: relative;
  }

  .surface-card__header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 0.5rem;
    margin-bottom: 0.25rem;
  }

  .surface-card__title {
    margin: 0;
    font-size: 0.9rem;
    letter-spacing: 0.08em;
    text-transform: uppercase;
    color: var(--metric-title-color);
  }

  .surface-card__toggle {
    appearance: none;
    border: 1px solid var(--control-surface-border);
    background: transparent;
    color: var(--lcars-text);
    font-size: 0.9rem;
    letter-spacing: 0.08em;
    text-transform: uppercase;
    padding: 0.35rem 0.65rem;
    border-radius: calc(var(--control-surface-radius) / 1.6);
    cursor: pointer;
    display: inline-flex;
    align-items: center;
    justify-content: center;
    gap: 0.35rem;
    position: relative;
  }

  .surface-card__toggle:hover,
  .surface-card__toggle:focus {
    background: rgba(255, 255, 255, 0.1);
    color: var(--lcars-accent, #f0f4ff);
    outline: none;
  }

  .surface-card__toggle:focus-visible {
    box-shadow: 0 0 0 2px rgba(0, 170, 255, 0.45);
  }

  .surface-card--collapsed .surface-card__content {
    display: none;
  }

  .surface-card--collapsed .surface-card__toggle {
    opacity: 0.85;
  }

  .surface-card__toggleIcon {
    font-size: 1.05rem;
    line-height: 1;
  }

  .surface-card__toggleLabel {
    display: inline-block;
    max-width: 0;
    opacity: 0;
    overflow: hidden;
    white-space: nowrap;
    transform: translateX(-0.25rem);
    transition:
      max-width 0.22s ease,
      opacity 0.22s ease,
      transform 0.22s ease;
  }

  .surface-card__toggle:hover .surface-card__toggleLabel,
  .surface-card__toggle:focus .surface-card__toggleLabel,
  .surface-card__toggle:focus-visible .surface-card__toggleLabel {
    max-width: 6rem;
    opacity: 1;
    transform: translateX(0);
  }

  .surface-card__toggleText {
    position: absolute;
    width: 1px;
    height: 1px;
    padding: 0;
    margin: -1px;
    overflow: hidden;
    clip: rect(0, 0, 0, 0);
    clip-path: inset(50%);
    border: 0;
    white-space: nowrap;
  }


  .surface-card__subtitle {
    margin: 0;
    font-size: 0.8rem;
    color: var(--lcars-muted);
  }

  /* (truncated — full styles kept in git history) */
`;
