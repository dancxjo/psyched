import type { ComponentChildren } from "preact";

import { type Accent, Card, Panel } from "./dashboard.tsx";
import { usePshStatus } from "@pilot/lib/psh_status.ts";

export interface ServiceOverviewProps {
  service: string;
  title: string;
  subtitle?: string;
  accent?: Accent;
  description?: string;
  docUrl?: string;
  children?: ComponentChildren;
}

/**
 * Present high-level lifecycle information about a dockerised service.
 *
 * @example
 * ```tsx
 * <ServiceOverview service="asr" title="ASR" description="Speech-to-text" />
 * ```
 */
export default function ServiceOverview({
  service,
  title,
  subtitle,
  accent = "teal",
  description,
  docUrl,
  children,
}: ServiceOverviewProps) {
  const status = usePshStatus("service", service);

  return (
    <Panel
      title={title}
      subtitle={subtitle}
      accent={accent}
      badges={[{
        label: status.label,
        tone: status.tone,
        pulse: status.loading,
      }]}
      actions={
        <a class="button button--ghost" href="/psh/srv">
          Open service console
        </a>
      }
    >
      <div class="panel-grid">
        <Card title="Lifecycle" tone={accent}>
          <dl class="stat-list">
            <div class="stat-list__item">
              <dt>Status</dt>
              <dd>{status.label}</dd>
            </div>
            <div class="stat-list__item">
              <dt>Identifier</dt>
              <dd>{service}</dd>
            </div>
            <div class="stat-list__item">
              <dt>Details</dt>
              <dd>{status.description ?? description ?? "—"}</dd>
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
            <a class="button button--ghost" href="/psh/srv">
              Manage services
            </a>
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
        {children}
      </div>
    </Panel>
  );
}
