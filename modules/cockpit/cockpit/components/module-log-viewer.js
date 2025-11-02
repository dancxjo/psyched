import {
  css,
  html,
  LitElement,
} from "https://unpkg.com/lit@3.1.4/index.js?module";
import { unsafeHTML } from "https://unpkg.com/lit@3.1.4/directives/unsafe-html.js?module";
import { AnsiUp } from "https://esm.sh/ansi_up@6.0.2";
import { surfaceStyles } from "./cockpit-style.js";
import { normaliseSystemdStatus } from "./cockpit-dashboard.helpers.js";
import { copyTextToClipboard, formatModuleLogForCopy } from "./log-copy.js";

/**
 * Compact module log viewer that surfaces the tail of a module's log file.
 *
 * The component fetches log entries from the cockpit backend and renders the
 * newest lines inside a scrollable panel. Operators can trigger manual
 * refreshes which abort any in-flight requests to keep the UI responsive.
 *
 * Example:
 * ```html
 * <cockpit-module-logs module="nav"></cockpit-module-logs>
 * ```
 */
class CockpitModuleLogs extends LitElement {
  static properties = {
    module: { type: String, reflect: true },
    moduleInfo: { attribute: false },
    lines: { state: true },
    truncated: { state: true },
    loading: { state: true },
    clearing: { state: true },
    errorMessage: { state: true },
    updatedAt: { state: true },
    moduleDetails: { state: true },
    systemdBusy: { state: true },
    systemdError: { state: true },
    copyState: { state: true },
    copyMessage: { state: true },
  };

  static styles = [
    surfaceStyles,
    css`
      :host {
        display: block;
      }

      .surface-log__meta {
        font-size: 0.75rem;
        color: var(--lcars-muted);
        display: flex;
        flex-wrap: wrap;
        gap: 0.5rem;
      }

      .surface-log__content {
        margin: 0;
        padding: 0.75rem 1rem;
        background: rgba(0, 0, 0, 0.45);
        border-radius: 0.5rem;
        max-height: 240px;
        overflow: auto;
        font-family: var(--metric-value-font);
        font-size: 0.65rem;
        line-height: 1.3;
        white-space: pre-wrap;
        word-break: break-word;
      }

      .surface-log__empty {
        font-size: 0.85rem;
        color: var(--lcars-muted);
        font-style: italic;
      }

      .surface-log__details {
        margin-top: 0.5rem;
      }

      .surface-log__summary {
        cursor: pointer;
        font-size: 0.85rem;
        font-weight: 600;
        color: var(--lcars-link, #f8c77c);
        display: inline-flex;
        align-items: center;
        gap: 0.35rem;
      }

      .module-log-panel .surface-card__content {
        display: grid;
        gap: 0.75rem;
      }

      .module-log-panel__header {
        display: flex;
        align-items: center;
        justify-content: space-between;
        gap: 0.5rem;
        flex-wrap: wrap;
      }

      .module-log-panel__title {
        display: flex;
        align-items: center;
        gap: 0.5rem;
      }

      .module-log-panel__heading {
        margin: 0;
        font-size: 0.85rem;
        letter-spacing: 0.08em;
        text-transform: uppercase;
        color: var(--metric-title-color);
      }

      .module-log-panel__actions {
        display: flex;
        flex-wrap: wrap;
        gap: 0.5rem;
        justify-content: flex-start;
      }

      .module-log-panel__actions .surface-action {
        font-size: 0.75rem;
        padding: 0.35rem 0.6rem;
      }

      .module-log-panel__controls {
        display: flex;
        flex-wrap: wrap;
        gap: 0.5rem;
      }

      .module-log-panel__controls .surface-action {
        font-size: 0.75rem;
        padding: 0.35rem 0.6rem;
      }

      .module-log-panel__meta {
        display: grid;
        gap: 0.35rem;
        font-size: 0.75rem;
        color: var(--lcars-muted);
      }

      .module-log-panel__metaRow {
        display: flex;
        flex-wrap: wrap;
        gap: 0.75rem;
        align-items: center;
      }

      .module-log-panel__metaLink {
        color: var(--lcars-accent);
        text-decoration: none;
      }

      .module-log-panel__log {
        display: grid;
        gap: 0.5rem;
      }

      .module-log-panel__subheading {
        margin: 0;
        font-size: 0.8rem;
        letter-spacing: 0.06em;
        text-transform: uppercase;
        color: var(--metric-label-color);
      }

      .module-log-panel__note,
      .module-log-panel__error {
        margin: 0;
        font-size: 0.75rem;
      }

      .module-log-panel__note {
        color: var(--lcars-muted);
      }

      .module-log-panel__error {
        color: var(--lcars-danger, #ff7f7f);
      }
    `,
  ];

  constructor() {
    super();
    this.module = "";
    this.moduleInfo = null;
    this.lines = [];
    this.truncated = false;
    this.loading = false;
    this.clearing = false;
    this.errorMessage = "";
    this.updatedAt = null;
    this.moduleDetails = this._normaliseModule(null);
    this.systemdBusy = "";
    this.systemdError = "";
    this.copyState = "idle";
    this.copyMessage = "";
    this._connected = false;
    this._abortController = null;
    this._lastFetchedModule = null;
    this._ansi = new AnsiUp();
    this._copyResetHandle = 0;
  }

  connectedCallback() {
    super.connectedCallback();
    this._connected = true;
    if (this.module && this._lastFetchedModule !== this.module) {
      this.refresh();
    }
  }

  disconnectedCallback() {
    super.disconnectedCallback();
    this._connected = false;
    this._abortFetch();
    this._clearCopyResetTimer();
  }

  updated(changedProperties) {
    if (!this._connected) {
      return;
    }
    if (changedProperties.has("moduleInfo")) {
      this.moduleDetails = this._normaliseModule(this.moduleInfo);
      this.systemdError = "";
    }
    if (changedProperties.has("module")) {
      if (!this.module) {
        this.lines = [];
        this.truncated = false;
        this.updatedAt = null;
        this.errorMessage = "";
        this._lastFetchedModule = null;
        this.moduleDetails = this._normaliseModule(null);
        this.systemdBusy = "";
        this.systemdError = "";
        this._setCopyState("idle");
        return;
      }
      if (this.module && this.module !== this._lastFetchedModule) {
        this.refresh();
      }
    }
  }

  render() {
    return this._renderModuleCommands();
  }

  _renderStatus() {
    if (!this.module) {
      return html`
        <p class="surface-status" data-variant="warning">Module not specified.</p>
      `;
    }
    if (this.errorMessage) {
      return html`
        <p class="surface-status" data-variant="error">${this.errorMessage}</p>
      `;
    }
    if (this.loading && !this.lines.length) {
      return html`
        <p class="surface-status">Loading log…</p>
      `;
    }
    if (!this.lines.length) {
      return html`
        <p class="surface-log__empty">No log entries captured yet.</p>
      `;
    }
    const ansiText = this.lines.join("\n");
    // Convert ANSI codes to HTML, then remove <br/> tags since we're using <pre>
    const htmlContent = ansiText
      ? this._ansi.ansi_to_html(ansiText).replace(/<br\s*\/?>/gi, "\n")
      : "";
    return html`
      <div class="surface-log__meta">
        <span>Showing ${this.lines.length} line${this.lines.length === 1
          ? ""
          : "s"}.</span>
        ${this.truncated
          ? html`
            <span>Older entries truncated.</span>
          `
          : ""} ${this.updatedAt
          ? html`
            <span>Updated ${this._formatTimestamp(this.updatedAt)}</span>
          `
          : html`
            <span>Awaiting first log write.</span>
          `}
      </div>
      <details class="surface-log__details">
        <summary class="surface-log__summary">View module log</summary>
        <pre class="surface-log__content">${unsafeHTML(htmlContent)}</pre>
      </details>
    `;
  }

  _renderModuleCommands() {
    const details = this.moduleDetails;
    const moduleKey = details.name || this.module;
    const slug = details.slug || moduleKey || "module";
    const scopeId = `module-${slug}`;
    const cardId = `module-commands-${slug}`;
    const systemd = details.systemd || normaliseSystemdStatus(null);
    const busyAction = this.systemdBusy;
    const errorMessage = this.systemdError;
    const canControl = Boolean(moduleKey) && systemd.supported;
    const activeVariant = systemd.active
      ? "success"
      : systemd.exists
      ? "warning"
      : "muted";
    const activeLabel = systemd.active
      ? "Active"
      : systemd.exists
      ? "Inactive"
      : "Missing";
    const enabledVariant = systemd.enabled ? "info" : "muted";
    const enabledLabel = systemd.enabled ? "Enabled" : "Disabled";

    if (!this.module) {
      return html`
        <article
          class="surface-card surface-card--collapsed module-log-panel"
          data-card-id="${cardId}"
          data-dashboard-scope="${scopeId}"
        >
          <header class="surface-card__header module-log-panel__header">
            <div class="module-log-panel__title">
              <h3 class="surface-card__title module-log-panel__heading">
                Module commands
              </h3>
            </div>
          </header>
          <div class="surface-card__content">
            <p class="module-log-panel__note">
              Set a module to manage lifecycle commands.
            </p>
            <div class="module-log-panel__log">
              <h4 class="module-log-panel__subheading">Module log</h4>
              ${this._renderStatus()}
            </div>
          </div>
        </article>
      `;
    }

    return html`
      <article
        class="surface-card surface-card--collapsed module-log-panel"
        data-card-id="${cardId}"
        data-dashboard-scope="${scopeId}"
      >
        <header class="surface-card__header module-log-panel__header">
          <div class="module-log-panel__title">
            <h3 class="surface-card__title module-log-panel__heading">
              Module commands
            </h3>
            <span class="surface-chip" data-variant="${details.hasCockpit
              ? "success"
              : "warning"}">
              ${details.hasCockpit ? "Dashboard ready" : "No dashboard"}
            </span>
          </div>
        </header>
        <div class="surface-card__content">
          <div class="module-log-panel__actions">
            <button
              type="button"
              class="surface-action"
              ?disabled="${this.loading || this.clearing || !this.module}"
              @click="${() => this._clearLogs()}"
            >
              ${this.clearing ? "Clearing…" : "Clear log"}
            </button>
            <button
              type="button"
              class="surface-action"
              ?disabled="${this.loading || this.clearing ||
                this.copyState === "copying" || !this.module}"
              @click="${() => this._copyLogs()}"
            >
              ${this.copyState === "copying"
                ? "Copying…"
                : this.copyState === "copied"
                ? "Copied!"
                : "Copy log"}
            </button>
            <button
              type="button"
              class="surface-action"
              ?disabled="${this.loading || this.clearing || !this.module}"
              @click="${() => this.refresh()}"
            >
              ${this.loading ? "Refreshing…" : "Refresh log"}
            </button>
          </div>
          <div class="module-log-panel__meta">
            <div class="module-log-panel__metaRow">
              <span>Module: ${details.displayName}</span>
              <span>Slug: ${details.slug || "n/a"}</span>
              ${details.dashboardUrl
                ? html`
                  <a
                    class="module-log-panel__metaLink"
                    href="${details.dashboardUrl}"
                    target="_blank"
                    rel="noreferrer"
                  >
                    Open dashboard
                  </a>
                `
                : ""} ${details.systemd.unit
                ? html`
                  <span>Unit: ${details.systemd.unit}</span>
                `
                : ""}
            </div>
            <div class="module-log-panel__metaRow">
              <span class="surface-chip" data-variant="${activeVariant}">${activeLabel}</span>
              <span class="surface-chip" data-variant="${enabledVariant}">${enabledLabel}</span>
            </div>
          </div>
          ${canControl
            ? html`
              <div class="module-log-panel__controls">
                <button
                  type="button"
                  class="surface-action"
                  @click="${() =>
                    this._runSystemdAction(systemd.active ? "down" : "up")}"
                  ?disabled="${Boolean(busyAction)}"
                >
                  ${busyAction === "up" || busyAction === "down"
                    ? "Working…"
                    : systemd.active
                    ? "Stop"
                    : "Start"}
                </button>
                <button
                  type="button"
                  class="surface-action"
                  @click="${() =>
                    this._runSystemdAction(
                      systemd.enabled ? "disable" : "enable",
                    )}"
                  ?disabled="${Boolean(busyAction)}"
                >
                  ${busyAction === "enable" || busyAction === "disable"
                    ? "Working…"
                    : systemd.enabled
                    ? "Disable"
                    : "Enable"}
                </button>
                ${systemd.exists
                  ? html`
                    <button
                      type="button"
                      class="surface-action"
                      @click="${() => this._runSystemdAction("teardown")}"
                      ?disabled="${Boolean(busyAction)}"
                    >
                      ${busyAction === "teardown" ? "Working…" : "Remove unit"}
                    </button>
                  `
                  : html`
                    <button
                      type="button"
                      class="surface-action"
                      @click="${() => this._runSystemdAction("setup")}"
                      ?disabled="${Boolean(busyAction)}"
                    >
                      ${busyAction === "setup" ? "Working…" : "Create unit"}
                    </button>
                  `}
                <button
                  type="button"
                  class="surface-action"
                  @click="${() => this._runSystemdAction("debug")}"
                  ?disabled="${Boolean(busyAction)}"
                >
                  ${busyAction === "debug" ? "Collecting…" : "Debug"}
                </button>
              </div>
            `
            : html`
              <p class="module-log-panel__note">
                Systemd integration unavailable on this host.
              </p>
            `} ${systemd.message
            ? html`
              <p class="module-log-panel__note">${systemd.message}</p>
            `
            : ""} ${errorMessage
            ? html`
              <p class="module-log-panel__error">${errorMessage}</p>
            `
            : ""} ${this.copyState === "failed" && this.copyMessage
            ? html`
              <p class="module-log-panel__error">${this.copyMessage}</p>
            `
            : ""}
          <div class="module-log-panel__log">
            <h4 class="module-log-panel__subheading">Module log</h4>
            ${this._renderStatus()}
          </div>
        </div>
      </article>
    `;
  }

  _normaliseModule(entry) {
    const module = entry && typeof entry === "object" ? entry : {};
    const rawName = typeof module.name === "string" && module.name.trim()
      ? module.name.trim()
      : "";
    const rawSlug = typeof module.slug === "string" && module.slug.trim()
      ? module.slug.trim()
      : "";
    const slug = rawSlug || rawName;
    const displayName =
      typeof module.display_name === "string" && module.display_name.trim()
        ? module.display_name.trim()
        : rawName || slug || "module";
    const hasCockpit = Boolean(module.has_cockpit);
    const rawDashboardUrl =
      typeof module.dashboard_url === "string" && module.dashboard_url.trim()
        ? module.dashboard_url.trim()
        : "";
    const dashboardUrl = rawDashboardUrl ||
      (hasCockpit && rawName ? `/modules/${rawName}/` : "");
    const systemd = normaliseSystemdStatus(module.systemd);

    return {
      name: rawName,
      slug,
      displayName,
      description: typeof module.description === "string"
        ? module.description.trim()
        : "",
      hasCockpit,
      dashboardUrl,
      systemd,
    };
  }

  async _copyLogs() {
    if (!this.module || this.copyState === "copying") {
      return;
    }
    this._setCopyState("copying");
    try {
      const payload = formatModuleLogForCopy(this.lines, {
        truncated: this.truncated,
        updatedAt: this.updatedAt,
      });
      const success = await copyTextToClipboard(payload);
      if (success) {
        this._setCopyState("copied");
      } else {
        this._setCopyState(
          "failed",
          "Unable to access the clipboard. Copy the log manually.",
        );
      }
    } catch (error) {
      const message = error instanceof Error ? error.message : String(error);
      this._setCopyState("failed", `Failed to copy module log: ${message}`);
    }
  }

  _setCopyState(state, message = "") {
    this.copyState = state;
    this.copyMessage = message;
    this._clearCopyResetTimer();
    if (state === "copied" || state === "failed") {
      this._copyResetHandle = globalThis.setTimeout(() => {
        this.copyState = "idle";
        this.copyMessage = "";
      }, 3000);
    }
  }

  _clearCopyResetTimer() {
    if (this._copyResetHandle) {
      globalThis.clearTimeout(this._copyResetHandle);
      this._copyResetHandle = 0;
    }
  }

  async _runSystemdAction(action) {
    if (!this.module || !action) {
      return;
    }
    this.systemdError = "";
    this.systemdBusy = action;

    try {
      const response = await fetch(
        `/api/modules/${encodeURIComponent(this.module)}/systemd/${
          encodeURIComponent(action)
        }`,
        {
          method: "POST",
          headers: { "Content-Type": "application/json" },
        },
      );

      let payload;
      try {
        payload = await response.json();
      } catch (_error) {
        payload = {};
      }

      if (!response.ok) {
        const message = payload && typeof payload.error === "string"
          ? payload.error
          : `Request failed with status ${response.status}`;
        throw new Error(message);
      }

      if (!payload.success) {
        const details =
          typeof payload.stderr === "string" && payload.stderr.trim()
            ? payload.stderr.trim()
            : typeof payload.stdout === "string" && payload.stdout.trim()
            ? payload.stdout.trim()
            : "Command did not complete successfully";
        throw new Error(details);
      }

      if (payload.status && typeof payload.status === "object") {
        const systemd = normaliseSystemdStatus(payload.status);
        this.moduleDetails = { ...this.moduleDetails, systemd };
        this.dispatchEvent(
          new CustomEvent("module-systemd-updated", {
            detail: { module: this.module, status: payload.status },
            bubbles: true,
            composed: true,
          }),
        );
      }

      if (action === "debug") {
        this._appendCommandOutput(action, payload);
      }
    } catch (error) {
      const message = error instanceof Error ? error.message : String(error);
      this.systemdError = message;
    } finally {
      this.systemdBusy = "";
    }
  }

  _appendCommandOutput(action, payload) {
    const timestamp = new Date();
    const header =
      `[module:${this.module}] command:${action} @ ${timestamp.toISOString()}`;
    const messages = [];
    if (
      payload && typeof payload.stdout === "string" && payload.stdout.trim()
    ) {
      messages.push(payload.stdout.trim());
    }
    if (
      payload && typeof payload.stderr === "string" && payload.stderr.trim()
    ) {
      messages.push(payload.stderr.trim());
    }
    const body = messages.length
      ? messages.join("\n")
      : "Command completed without output.";
    const block = `${header}\n${body}`;
    const newLines = block.split(/\r?\n/);
    this.lines = [...newLines, ...this.lines];
    this.truncated = false;
    this.updatedAt = timestamp.toISOString();
  }

  async refresh() {
    if (!this.module) {
      return;
    }
    this._abortFetch();
    const controller = new AbortController();
    this._abortController = controller;
    this._lastFetchedModule = this.module;
    this.loading = true;
    this.errorMessage = "";

    try {
      const response = await fetch(
        `/api/modules/${encodeURIComponent(this.module)}/logs`,
        {
          signal: controller.signal,
        },
      );
      if (!response.ok) {
        throw new Error(`Request failed with status ${response.status}`);
      }
      const payload = await response.json();
      this.lines = Array.isArray(payload.lines) ? payload.lines : [];
      this.truncated = Boolean(payload.truncated);
      this.updatedAt = typeof payload.updated_at === "string"
        ? payload.updated_at
        : null;
    } catch (error) {
      if (controller.signal.aborted) {
        return;
      }
      const message = error instanceof Error ? error.message : String(error);
      this.errorMessage = `Failed to load module log: ${message}`;
      this.lines = [];
      this.truncated = false;
      this.updatedAt = null;
    } finally {
      if (this._abortController === controller) {
        this._abortController = null;
      }
      this.loading = false;
    }
  }

  async _clearLogs() {
    if (!this.module || this.clearing) {
      return;
    }
    this._abortFetch();
    this.clearing = true;
    this.errorMessage = "";

    try {
      const response = await fetch(
        `/api/modules/${encodeURIComponent(this.module)}/logs`,
        {
          method: "DELETE",
        },
      );
      if (!response.ok) {
        throw new Error(`Request failed with status ${response.status}`);
      }
      this.lines = [];
      this.truncated = false;
      this.updatedAt = null;
      await this.refresh();
    } catch (error) {
      const message = error instanceof Error ? error.message : String(error);
      this.errorMessage = `Failed to clear module log: ${message}`;
    } finally {
      this.clearing = false;
    }
  }

  _abortFetch() {
    if (this._abortController) {
      this._abortController.abort();
      this._abortController = null;
    }
  }

  _formatTimestamp(value) {
    try {
      const date = new Date(value);
      if (Number.isNaN(date.getTime())) {
        return value;
      }
      return `${date.toLocaleDateString()} ${date.toLocaleTimeString()}`;
    } catch (_error) {
      return value;
    }
  }
}

customElements.define("cockpit-module-logs", CockpitModuleLogs);
