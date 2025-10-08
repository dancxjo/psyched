import { useCallback, useMemo, useState } from "preact/hooks";
import type { ComponentChildren } from "preact";

import { type Accent, Card, Panel } from "./dashboard.tsx";
import {
  MODULE_ACTIONS,
  type ModuleAction,
  moduleActionEndpoint,
  moduleActionLabel,
} from "./module_actions.ts";
import { usePshStatus } from "@pilot/lib/psh_status.ts";
import { formatRelativeTime } from "../../frontend/lib/format.ts";

interface ModuleOverviewProps {
  /** Identifier used by `psh` to reference the module. */
  moduleName: string;
  /** Heading displayed at the top of the panel. */
  title: string;
  /** Optional supporting copy rendered under the title. */
  description?: string;
  /** Accent applied to the panel and action buttons. */
  accent?: Accent;
  /** Link to module documentation surfaced beside the quick actions. */
  docUrl?: string;
  /** Additional cards rendered alongside the default lifecycle controls. */
  children?: ComponentChildren;
}

type ActionMessage =
  | { kind: "success"; text: string }
  | { kind: "error"; text: string };

const SUCCESS_MESSAGE = "Lifecycle command dispatched successfully.";
const ERROR_PREFIX = "Lifecycle command failed:";

/**
 * Present lifecycle telemetry and quick controls for a ROS module.
 *
 * @example
 * ```tsx
 * <ModuleOverview
 *   moduleName="chat"
 *   title="Chat module"
 *   description="Conversational agent backed by the LLM service."
 * />
 * ```
 */
export default function ModuleOverview({
  moduleName,
  title,
  description,
  accent = "teal",
  docUrl,
  children,
}: ModuleOverviewProps) {
  const status = usePshStatus("module", moduleName);
  const [pendingAction, setPendingAction] = useState<ModuleAction | null>(
    null,
  );
  const [message, setMessage] = useState<ActionMessage | null>(null);

  const lastUpdatedLabel = useMemo(
    () => formatRelativeTime(status.lastUpdated),
    [status.lastUpdated],
  );

  const handleAction = useCallback(
    async (action: ModuleAction) => {
      if (typeof fetch !== "function") {
        return;
      }

      setPendingAction(action);
      setMessage(null);
      try {
        const response = await fetch(moduleActionEndpoint(action), {
          method: "POST",
          headers: { "content-type": "application/json" },
          body: JSON.stringify({ modules: [moduleName] }),
        });
        const payload = await response.json() as {
          ok: boolean;
          error?: string;
        };
        if (!payload.ok) {
          throw new Error(payload.error ?? "Unknown lifecycle error");
        }
        setMessage({ kind: "success", text: SUCCESS_MESSAGE });
        status.refresh();
      } catch (error) {
        const text = error instanceof Error ? error.message : String(error);
        setMessage({ kind: "error", text: `${ERROR_PREFIX} ${text}` });
      } finally {
        setPendingAction(null);
      }
    },
    [moduleName, status.refresh],
  );

  return (
    <Panel
      title={title}
      subtitle={description}
      accent={accent}
      badges={[
        {
          label: status.label,
          tone: status.tone,
          pulse: status.loading,
        },
        {
          label: `Updated ${lastUpdatedLabel}`,
          tone: "neutral",
        },
      ]}
      actions={
        <a class="button button--ghost" href="/psh/mod">
          Open module console
        </a>
      }
    >
      <div class="panel-grid panel-grid--stretch">
        <Card title="Lifecycle" subtitle="psh mod status" tone={accent}>
          <dl class="stat-list">
            <div class="stat-list__item">
              <dt>Status</dt>
              <dd>{status.label}</dd>
            </div>
            <div class="stat-list__item">
              <dt>Last updated</dt>
              <dd>{lastUpdatedLabel}</dd>
            </div>
            <div class="stat-list__item">
              <dt>Details</dt>
              <dd>{status.description ?? "—"}</dd>
            </div>
            {status.error && (
              <div class="stat-list__item">
                <dt>Last error</dt>
                <dd>{status.error}</dd>
              </div>
            )}
          </dl>
          <div class="button-group button-group--wrap">
            <button
              class="button button--primary"
              type="button"
              onClick={() => status.refresh()}
              disabled={status.loading}
            >
              Refresh status
            </button>
            {docUrl && (
              <a
                class="button button--ghost"
                href={docUrl}
                target="_blank"
                rel="noreferrer"
              >
                Documentation
              </a>
            )}
          </div>
        </Card>

        <Card title="Controls" subtitle="psh mod lifecycle" tone="neutral">
          {message && (
            <p
              class={`note ${
                message.kind === "success" ? "note--success" : "note--alert"
              }`}
            >
              {message.text}
            </p>
          )}
          <div class="button-group button-group--wrap">
            {MODULE_ACTIONS.map((action) => (
              <button
                key={action}
                class={`button ${
                  action === "teardown"
                    ? "button--danger"
                    : action === "down"
                    ? "button--ghost"
                    : "button--primary"
                }`}
                type="button"
                disabled={pendingAction !== null}
                onClick={() => handleAction(action)}
              >
                {moduleActionLabel(action)}
              </button>
            ))}
          </div>
        </Card>

        {children}
      </div>
    </Panel>
  );
}
