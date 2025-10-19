import { LitElement, html, css } from 'https://unpkg.com/lit@3.1.4/index.js?module';
import { buildNavigationSections } from '/utils/navigation.js';
import './module-log-viewer.js';
import { surfaceStyles } from './cockpit-style.js';
import { normaliseSystemdStatus } from './cockpit-dashboard.helpers.js';

// Component registry - maps module names to their component tag names
const MODULE_COMPONENTS = {
  chat: 'chat-dashboard',
  ear: 'ear-dashboard',
  eye: 'eye-dashboard',
  faces: 'faces-dashboard',
  pilot: 'pilot-dashboard',
  foot: 'foot-dashboard',
  gps: 'gps-dashboard',
  hypothalamus: 'hypothalamus-dashboard',
  imu: 'imu-dashboard',
  memory: 'memory-dashboard',
  nav: 'nav-dashboard',
  cockpit: 'cockpit-dashboard',
  viscera: 'viscera-dashboard',
  voice: 'voice-dashboard',
};

/**
 * Lightweight shell that lists module dashboards surfaced by the cockpit.
 */
class CockpitApp extends LitElement {
  static styles = [
    surfaceStyles,
    css`
      .module-surface {
        display: grid;
        gap: 0.85rem;
        grid-template-columns: minmax(0, 1fr);
        align-items: start;
      }

      .module-surface__primary {
        grid-column: 1 / -1;
      }

      .module-surface__logs {
        grid-column: 1 / -1;
      }

      .module-meta-card {
        grid-column: 1 / -1;
        justify-self: stretch;
        width: 100%;
      }

      .module-meta-card__header {
        display: flex;
        align-items: center;
        justify-content: space-between;
        gap: 0.75rem;
      }

      .module-meta-card__name {
        font-family: var(--metric-value-font);
        font-size: 1.05rem;
      }

      .module-meta-card__meta {
        display: flex;
        flex-wrap: wrap;
        gap: 0.75rem;
        font-size: 0.75rem;
        color: var(--lcars-muted);
      }

      .module-meta-card__link {
        color: var(--lcars-accent);
        text-decoration: none;
      }

      .module-meta-card__systemd {
        display: flex;
        flex-wrap: wrap;
        gap: 0.5rem;
        align-items: center;
      }

      .module-meta-card__actions {
        display: flex;
        flex-wrap: wrap;
        gap: 0.5rem;
      }

      .module-meta-card__actions .surface-action {
        font-size: 0.75rem;
        padding: 0.35rem 0.6rem;
      }

      .module-meta-card__note,
      .module-meta-card__error {
        margin: 0;
        font-size: 0.75rem;
      }

      .module-meta-card__note {
        color: var(--lcars-muted);
      }

      .module-meta-card__error {
        color: var(--lcars-danger, #ff7f7f);
      }

      @media (min-width: 960px) {
        .module-surface {
          grid-template-columns: minmax(0, 2fr) minmax(0, 1fr);
        }

        .module-surface__logs {
          grid-column: 1 / 2;
        }

        .module-meta-card {
          grid-column: 2 / 3;
          justify-self: end;
          max-width: 420px;
        }
      }
    `,
  ];

  static properties = {
    modules: { state: true },
    loading: { state: true },
    errorMessage: { state: true },
    systemdBusy: { state: true },
    systemdErrors: { state: true },
  };

  constructor() {
    super();
    this.modules = [];
    this.loading = true;
    this.errorMessage = '';
    this.loadedComponents = new Set();
    this.systemdBusy = {};
    this.systemdErrors = {};
  }

  createRenderRoot() {
    return this;
  }

  connectedCallback() {
    super.connectedCallback();
    this.refresh();
  }

  async refresh() {
    this.loading = true;
    this.errorMessage = '';
    try {
      const response = await fetch('/api/modules');
      if (!response.ok) {
        throw new Error(`Request failed with status ${response.status}`);
      }
      const payload = await response.json();
      this.modules = Array.isArray(payload.modules) ? payload.modules : [];
      this.systemdBusy = {};
      this.systemdErrors = {};
      this.broadcastNavigation();
      this.updateHostGlobals(payload.host);
      this.updateBridgeGlobals(payload.bridge);
      await this.loadModuleComponents();
    } catch (error) {
      this.errorMessage = error instanceof Error ? error.message : String(error);
    } finally {
      this.loading = false;
    }
  }

  async loadModuleComponents() {
    for (const module of this.modules) {
      const componentTag = MODULE_COMPONENTS[module.name];
      if (componentTag && !this.loadedComponents.has(module.name)) {
        try {
          await import(`/modules/${module.name}/components/${componentTag}.js`);
          this.loadedComponents.add(module.name);
        } catch (error) {
          console.warn(`Failed to load component for module ${module.name}:`, error);
        }
      }
    }
  }

  updateBridgeGlobals(bridge) {
    if (typeof window === 'undefined') {
      return;
    }
    const cockpitGlobals = window.Cockpit ? { ...window.Cockpit } : {};
    cockpitGlobals.bridge = {
      ...(cockpitGlobals.bridge || {}),
      ...(bridge || {}),
    };
    window.Cockpit = cockpitGlobals;
  }

  updateHostGlobals(host) {
    if (typeof window === 'undefined' || !host || typeof host !== 'object') {
      return;
    }
    const cockpitGlobals = window.Cockpit ? { ...window.Cockpit } : {};
    cockpitGlobals.host = {
      ...(cockpitGlobals.host || {}),
      ...host,
    };
    window.Cockpit = cockpitGlobals;
  }

  broadcastNavigation() {
    if (typeof window === 'undefined') {
      return;
    }

    const sections = buildNavigationSections(this.modules);
    const cockpitGlobals = window.Cockpit ? { ...window.Cockpit } : {};
    cockpitGlobals.navigation = {
      ...(cockpitGlobals.navigation || {}),
      sections,
    };
    window.Cockpit = cockpitGlobals;

    window.dispatchEvent(new CustomEvent('cockpit-sections', { detail: sections }));
  }

  render() {
    if (this.loading) {
      return html`<section class="cockpit-loading">Loading module dashboards…</section>`;
    }
    if (this.errorMessage) {
      return html`<section class="cockpit-error">
        <h2>Failed to load modules</h2>
        <p>${this.errorMessage}</p>
        <button type="button" @click=${() => this.refresh()}>Retry</button>
      </section>`;
    }
    if (!this.modules.length) {
      return html`<section class="cockpit-empty">
        <h2>No modules are currently active</h2>
        <p>The host configuration does not surface any cockpit dashboards. Confirm host.modules is populated.</p>
      </section>`;
    }

    return html`
      <div class="module-stack">
        ${this.modules.map((module) => this.renderModuleSurface(module))}
      </div>
    `;
  }

  renderModuleSurface(module) {
    const slug = module.slug || module.name;
    const displayName = module.display_name || module.name;
    const componentTag = MODULE_COMPONENTS[module.name];
    const normalisedModule = this._normaliseModule(module);
    const moduleContent = componentTag
      ? this.renderComponent(componentTag)
      : html`<div class="module-placeholder">
          <p>This module does not expose cockpit assets yet.</p>
        </div>`;

    return html`
      <section class="module-section" id=${`module-${slug}`}>
        <header class="module-section__header">
          <h2>${displayName}</h2>
          ${module.description ? html`<p class="module-description">${module.description}</p>` : ''}
        </header>
        <div class="module-surface surface-grid surface-grid--stack">
          <div class="module-surface__primary">
            ${moduleContent}
          </div>
          <cockpit-module-logs class="module-surface__logs" module=${module.name}></cockpit-module-logs>
          ${this._renderModuleMeta(normalisedModule)}
        </div>
      </section>
    `;
  }

  _normaliseModule(entry) {
    const module = entry && typeof entry === 'object' ? entry : {};
    const rawName = typeof module.name === 'string' && module.name.trim() ? module.name.trim() : '';
    const rawSlug = typeof module.slug === 'string' && module.slug.trim() ? module.slug.trim() : '';
    const slug = rawSlug || rawName;
    const displayName = typeof module.display_name === 'string' && module.display_name.trim()
      ? module.display_name.trim()
      : rawName || slug || 'module';
    const description = typeof module.description === 'string' ? module.description.trim() : '';
    const hasCockpit = Boolean(module.has_cockpit);
    const rawDashboardUrl = typeof module.dashboard_url === 'string' && module.dashboard_url.trim()
      ? module.dashboard_url.trim()
      : '';
    const dashboardUrl = rawDashboardUrl || (hasCockpit && rawName ? `/modules/${rawName}/` : '');
    const systemd = normaliseSystemdStatus(module.systemd);

    return { name: rawName, slug, displayName, description, hasCockpit, dashboardUrl, systemd };
  }

  _renderModuleMeta(module) {
    const moduleKey = module.name || module.slug;
    const systemd = module.systemd;
    const busyAction = moduleKey ? this.systemdBusy[moduleKey] || '' : '';
    const errorMessage = moduleKey ? this.systemdErrors[moduleKey] || '' : '';
    const canControl = Boolean(moduleKey) && systemd.supported;
    const activeVariant = systemd.active ? 'success' : systemd.exists ? 'warning' : 'muted';
    const activeLabel = systemd.active ? 'Active' : systemd.exists ? 'Inactive' : 'Missing';
    const enabledVariant = systemd.enabled ? 'info' : 'muted';
    const enabledLabel = systemd.enabled ? 'Enabled' : 'Disabled';

    return html`
      <article class="surface-card surface-card--compact module-meta-card">
        <div class="module-meta-card__header">
          <span class="module-meta-card__name">${module.displayName}</span>
          <span class="surface-chip" data-variant=${module.hasCockpit ? 'success' : 'warning'}>
            ${module.hasCockpit ? 'Dashboard ready' : 'No dashboard'}
          </span>
        </div>
        <div class="module-meta-card__meta">
          <span>Slug: ${module.slug || 'n/a'}</span>
          ${module.dashboardUrl
            ? html`<a class="module-meta-card__link" href="${module.dashboardUrl}" target="_blank" rel="noreferrer">Open dashboard</a>`
            : ''}
        </div>
        <div class="module-meta-card__systemd">
          <span class="surface-chip" data-variant=${activeVariant}>${activeLabel}</span>
          <span class="surface-chip" data-variant=${enabledVariant}>${enabledLabel}</span>
          ${systemd.unit ? html`<span class="surface-status">${systemd.unit}</span>` : ''}
        </div>
        ${canControl
          ? html`<div class="module-meta-card__actions">
              <button
                type="button"
                class="surface-action"
                @click=${() => this._runSystemdAction(moduleKey, systemd.active ? 'down' : 'up')}
                ?disabled=${Boolean(busyAction)}
              >
                ${busyAction === 'up' || busyAction === 'down'
                  ? 'Working…'
                  : systemd.active ? 'Stop' : 'Start'}
              </button>
              <button
                type="button"
                class="surface-action"
                @click=${() => this._runSystemdAction(moduleKey, systemd.enabled ? 'disable' : 'enable')}
                ?disabled=${Boolean(busyAction)}
              >
                ${busyAction === 'enable' || busyAction === 'disable'
                  ? 'Working…'
                  : systemd.enabled ? 'Disable' : 'Enable'}
              </button>
              ${systemd.exists
                ? html`<button
                      type="button"
                      class="surface-action"
                      @click=${() => this._runSystemdAction(moduleKey, 'teardown')}
                      ?disabled=${Boolean(busyAction)}
                    >${busyAction === 'teardown' ? 'Working…' : 'Remove unit'}</button>`
                : html`<button
                      type="button"
                      class="surface-action"
                      @click=${() => this._runSystemdAction(moduleKey, 'setup')}
                      ?disabled=${Boolean(busyAction)}
                    >${busyAction === 'setup' ? 'Working…' : 'Create unit'}</button>`}
              <button
                type="button"
                class="surface-action"
                @click=${() => this._runSystemdAction(moduleKey, 'debug')}
                ?disabled=${Boolean(busyAction)}
              >
                ${busyAction === 'debug' ? 'Collecting…' : 'Debug'}
              </button>
            </div>`
          : html`<p class="module-meta-card__note">Systemd integration unavailable on this host.</p>`}
        ${systemd.message ? html`<p class="module-meta-card__note">${systemd.message}</p>` : ''}
        ${errorMessage ? html`<p class="module-meta-card__error">${errorMessage}</p>` : ''}
      </article>
    `;
  }

  async _runSystemdAction(moduleName, action) {
    if (!moduleName || !action) {
      return;
    }

    this.systemdErrors = { ...this.systemdErrors, [moduleName]: '' };
    this.systemdBusy = { ...this.systemdBusy, [moduleName]: action };

    try {
      const response = await fetch(
        `/api/modules/${encodeURIComponent(moduleName)}/systemd/${encodeURIComponent(action)}`,
        {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
        },
      );

      let payload;
      try {
        payload = await response.json();
      } catch (_error) {
        payload = {};
      }

      if (!response.ok) {
        const message = payload && typeof payload.error === 'string'
          ? payload.error
          : `Request failed with status ${response.status}`;
        throw new Error(message);
      }

      if (!payload.success) {
        const details = typeof payload.stderr === 'string' && payload.stderr.trim()
          ? payload.stderr.trim()
          : typeof payload.stdout === 'string' && payload.stdout.trim()
            ? payload.stdout.trim()
            : 'Command did not complete successfully';
        throw new Error(details);
      }

      if (payload.status && typeof payload.status === 'object') {
        this.modules = this.modules.map((entry) => {
          if (!entry || typeof entry !== 'object') {
            return entry;
          }
          const entryName = typeof entry.name === 'string' ? entry.name : '';
          const entrySlug = typeof entry.slug === 'string' ? entry.slug : '';
          if (entryName === moduleName || (!entryName && entrySlug === moduleName)) {
            return { ...entry, systemd: payload.status };
          }
          return entry;
        });
      }

      if (typeof payload.stdout === 'string' && payload.stdout.trim()) {
        console.info(`[cockpit] ${moduleName} ${action}: ${payload.stdout.trim()}`);
      }
    } catch (error) {
      const message = error instanceof Error ? error.message : String(error);
      this.systemdErrors = { ...this.systemdErrors, [moduleName]: message };
    } finally {
      const { [moduleName]: _removed, ...rest } = this.systemdBusy;
      this.systemdBusy = rest;
    }
  }

  renderComponent(tagName) {
    // Create the component element dynamically based on tag name
    const tagMap = {
      'chat-dashboard': html`<chat-dashboard></chat-dashboard>`,
      'ear-dashboard': html`<ear-dashboard></ear-dashboard>`,
      'eye-dashboard': html`<eye-dashboard></eye-dashboard>`,
      'faces-dashboard': html`<faces-dashboard></faces-dashboard>`,
      'pilot-dashboard': html`<pilot-dashboard></pilot-dashboard>`,
      'foot-dashboard': html`<foot-dashboard></foot-dashboard>`,
      'gps-dashboard': html`<gps-dashboard></gps-dashboard>`,
      'hypothalamus-dashboard': html`<hypothalamus-dashboard></hypothalamus-dashboard>`,
      'imu-dashboard': html`<imu-dashboard></imu-dashboard>`,
      'memory-dashboard': html`<memory-dashboard></memory-dashboard>`,
      'nav-dashboard': html`<nav-dashboard></nav-dashboard>`,
      'cockpit-dashboard': html`<cockpit-dashboard></cockpit-dashboard>`,
      'viscera-dashboard': html`<viscera-dashboard></viscera-dashboard>`,
      'voice-dashboard': html`<voice-dashboard></voice-dashboard>`,
    };
    return tagMap[tagName] || html`<p>Component ${tagName} not found</p>`;
  }
}

customElements.define('cockpit-app', CockpitApp);
