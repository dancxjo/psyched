import {
  css,
  html,
  LitElement,
} from "https://unpkg.com/lit@3.1.4/index.js?module";
import { surfaceStyles } from "/components/cockpit-style.js";
import {
  normaliseBridgeSettings,
  normaliseHostMetadata,
  summariseModules,
} from "./cockpit-dashboard.helpers.js";

/**
 * Cockpit operations console that summarises host metadata and bridge endpoints.
 *
 * The dashboard provides a quick snapshot of the cockpit's own health so
 * operators can confirm rosbridge and video relays before diving into other
 * module consoles.
 *
 * @example
 * ```html
 * <cockpit-dashboard></cockpit-dashboard>
 * ```
 */
class CockpitDashboard extends LitElement {
  static properties = {
    loading: { state: true },
    errorMessage: { state: true },
    host: { state: true },
    bridge: { state: true },
    summary: { state: true },
    lastUpdated: { state: true },
    hostActionPending: { state: true },
    hostActionStatus: { state: true },
    hostActionVariant: { state: true },
  };

  static styles = [
    surfaceStyles,
    css`
      :host {
        display: block;
      }

      .dashboard-status {
        display: flex;
        align-items: center;
        justify-content: space-between;
        gap: 0.75rem;
      }

      .dashboard-status__meta {
        display: flex;
        flex-direction: column;
        gap: 0.25rem;
      }

      .dashboard-status__actions {
        display: flex;
        align-items: center;
      }

      .metric-grid {
        display: grid;
        grid-template-columns: repeat(auto-fit, minmax(180px, 1fr));
        gap: 0.75rem;
      }

      .host-actions {
        display: flex;
        flex-wrap: wrap;
        gap: 0.4rem;
        margin-top: 0.35rem;
      }

      .surface-action[data-variant="danger"] {
        background: rgba(255, 111, 97, 0.25);
        color: var(--lcars-error);
      }

      .surface-action[data-variant="danger"]:hover,
      .surface-action[data-variant="danger"]:focus {
        background: rgba(255, 111, 97, 0.4);
      }

      .surface-action[data-variant="warning"] {
        background: rgba(255, 192, 76, 0.25);
        color: var(--lcars-warning);
      }

      .surface-action[data-variant="warning"]:hover,
      .surface-action[data-variant="warning"]:focus {
        background: rgba(255, 192, 76, 0.4);
      }
    `,
  ];

  constructor() {
    super();
    this.loading = true;
    this.errorMessage = "";
    this.host = normaliseHostMetadata(null);
    this.bridge = normaliseBridgeSettings(null);
    this.summary = summariseModules([]);
    this.lastUpdated = null;
    this.hostActionPending = "";
    this.hostActionStatus = "";
    this.hostActionVariant = "";
    this._abortController = null;
  }

  connectedCallback() {
    super.connectedCallback();
    this.refresh();
  }

  disconnectedCallback() {
    super.disconnectedCallback();
    this._abortFetch();
  }

  render() {
    return html`
      <div class="surface-grid surface-grid--stack">
        <article class="surface-card">
          <header class="dashboard-status">
            <div class="dashboard-status__meta">
              <h3 class="surface-card__title">Cockpit overview</h3>
              ${this._renderStatus()}
            </div>
            <div class="dashboard-status__actions">
              <button
                type="button"
                class="surface-action"
                ?disabled="${this.loading}"
                @click="${() => this.refresh()}"
              >
                ${this.loading ? "Refreshing…" : "Refresh"}
              </button>
            </div>
          </header>
          <div class="metric-grid">
            ${this._renderHostCard()} ${this._renderBridgeCard()} ${this
              ._renderModuleSummaryCard()}
          </div>
        </article>
      </div>
    `;
  }

  _renderStatus() {
    if (this.errorMessage) {
      return html`
        <p class="surface-status" data-variant="error">${this.errorMessage}</p>
      `;
    }
    if (this.loading) {
      return html`
        <p class="surface-status">Loading cockpit metadata…</p>
      `;
    }
    const timestamp = this.lastUpdated instanceof Date
      ? this.lastUpdated.toLocaleTimeString()
      : "Never";
    return html`
      <p class="surface-status">Updated ${timestamp}</p>
    `;
  }

  _renderHostCard() {
    const disableActions = this.loading || Boolean(this.hostActionPending);
    const shutdownLabel = this.hostActionPending === "shutdown"
      ? "Shutting down…"
      : "Shutdown";
    const restartLabel = this.hostActionPending === "restart"
      ? "Restarting…"
      : "Restart";
    return html`
      <section class="surface-metric">
        <span class="surface-metric__label">Host</span>
        <span class="surface-metric__value surface-metric__value--large"
        >${this.host.name}</span>
        <span class="surface-status">Shortname: ${this.host.shortname}</span>
        <div class="host-actions">
          <button
            type="button"
            class="surface-action"
            data-variant="danger"
            ?disabled="${disableActions}"
            @click="${() => this._confirmHostOperation("shutdown")}"
          >
            ${shutdownLabel}
          </button>
          <button
            type="button"
            class="surface-action"
            data-variant="warning"
            ?disabled="${disableActions}"
            @click="${() => this._confirmHostOperation("restart")}"
          >
            ${restartLabel}
          </button>
        </div>
        ${this.hostActionStatus
          ? html`
            <p class="surface-status" data-variant="${this.hostActionVariant ||
              undefined}">${this.hostActionStatus}</p>
          `
          : ""}
      </section>
    `;
  }

  _renderBridgeCard() {
    return html`
      <section class="surface-metric">
        <span class="surface-metric__label">ROS Bridge</span>
        <span class="surface-metric__value">${this.bridge.mode}</span>
        <span class="surface-status">Primary: ${this.bridge
          .effectiveRosbridgeUri}</span>
        ${this.bridge.videoBase
          ? html`
            <span class="surface-status">
              Video base:
              <a
                href="${this.bridge.videoUrl || this.bridge.videoBase}"
                title="${this.bridge.videoBase}"
                target="_blank"
                rel="noopener noreferrer"
              >
                Cockpit overview
              </a>
            </span>
          `
          : ""}
      </section>
    `;
  }

  _renderModuleSummaryCard() {
    const { total, withCockpit, withoutCockpit } = this.summary;
    return html`
      <section class="surface-metric">
        <span class="surface-metric__label">Modules</span>
        <span class="surface-metric__value surface-metric__value--large"
        >${total}</span>
        <span class="surface-status">Dashboards ready: ${withCockpit}</span>
        <span class="surface-status">Awaiting dashboards: ${withoutCockpit}</span>
      </section>
    `;
  }

  async refresh() {
    this._abortFetch();
    const controller = new AbortController();
    this._abortController = controller;
    this.loading = true;
    this.errorMessage = "";

    try {
      const response = await fetch("/api/modules", {
        signal: controller.signal,
      });
      if (!response.ok) {
        throw new Error(`Request failed with status ${response.status}`);
      }
      const payload = await response.json();
      this.host = normaliseHostMetadata(payload.host);
      this.bridge = normaliseBridgeSettings(payload.bridge);
      const modules = Array.isArray(payload.modules) ? payload.modules : [];
      this.summary = summariseModules(modules);
      this.lastUpdated = new Date();
    } catch (error) {
      if (controller.signal.aborted) {
        return;
      }
      this.errorMessage = error instanceof Error
        ? error.message
        : String(error);
    } finally {
      if (this._abortController === controller) {
        this._abortController = null;
      }
      this.loading = false;
    }
  }

  _abortFetch() {
    if (this._abortController) {
      this._abortController.abort();
      this._abortController = null;
    }
  }

  async _confirmHostOperation(operation) {
    const canonical = operation === "restart" ? "restart" : "shutdown";
    const verb = canonical === "restart" ? "restart" : "shut down";
    const hostName = this.host && this.host.name ? this.host.name : "the host";
    if (
      typeof window === "undefined" ||
      window.confirm(`Are you sure you want to ${verb} ${hostName}?`)
    ) {
      await this._runHostOperation(canonical);
    }
  }

  async _runHostOperation(operation) {
    const canonical = operation === "restart" ? "restart" : "shutdown";
    const label = canonical === "restart" ? "Restart" : "Shutdown";
    this.hostActionPending = canonical;
    this.hostActionStatus = `Dispatching ${label.toLowerCase()} command…`;
    this.hostActionVariant = "";

    try {
      const response = await fetch("/api/ops/host-power", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ operation: canonical }),
      });
      if (!response.ok) {
        const message = await response.text();
        throw new Error(
          message || `Request failed with status ${response.status}`,
        );
      }
      const payload = await response.json();
      if (!payload || typeof payload !== "object") {
        throw new Error(`${label} command returned an unexpected response.`);
      }
      if (payload.success === false) {
        const stderr = typeof payload.stderr === "string"
          ? payload.stderr.trim()
          : "";
        const detail = stderr ||
          `${label} command failed with exit code ${
            payload.returncode ?? "unknown"
          }.`;
        throw new Error(detail);
      }
      this.hostActionStatus =
        `${label} command dispatched. The host may disconnect shortly.`;
      this.hostActionVariant = "success";
    } catch (error) {
      const message = error instanceof Error ? error.message : String(error);
      const lowered = message.toLowerCase();
      const maybeNetwork = lowered.includes("failed to fetch") ||
        lowered.includes("network");
      this.hostActionStatus = maybeNetwork
        ? `${label} response interrupted; the host may already be cycling.`
        : (message || `Unable to ${label.toLowerCase()} the host.`);
      this.hostActionVariant = maybeNetwork ? "warning" : "error";
    } finally {
      this.hostActionPending = "";
    }
  }
}

customElements.define("cockpit-dashboard", CockpitDashboard);
