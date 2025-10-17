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

  /* (truncated — full styles kept in git history) */
`;
