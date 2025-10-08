import { useMemo } from "preact/hooks";

import { Badge } from "@pilot/components/dashboard.tsx";
import { usePshStatus } from "@pilot/lib/psh_status.ts";

export interface DashboardStatusBadgeProps {
  kind: "module" | "service";
  name: string;
  refreshIntervalMs?: number;
}

/**
 * Live badge summarising the lifecycle state of a module or service.
 *
 * The badge polls the relevant `/api/psh` endpoint and surfaces the status as a
 * colour-coded pill. When a fetch error occurs the badge switches to the
 * "danger" palette and surfaces the error text below the badge.
 */
export default function DashboardStatusBadge({
  kind,
  name,
  refreshIntervalMs,
}: DashboardStatusBadgeProps) {
  const status = usePshStatus(kind, name, { refreshIntervalMs });
  const message = useMemo(() => {
    if (status.error) return status.error;
    if (status.description && status.description !== status.label) {
      return status.description;
    }
    return undefined;
  }, [status.description, status.error, status.label]);

  return (
    <div class="dashboard-status-badge">
      <Badge label={status.label} tone={status.tone} pulse={status.loading} />
      {message && <span class="dashboard-status-badge__note">{message}</span>}
    </div>
  );
}
