import { useMemo } from "preact/hooks";

import { Badge } from "@pilot/components/dashboard.tsx";
import { usePshStatus } from "@pilot/lib/psh_status.ts";

/**
 * Polling interval used by the pilot dashboard to keep lifecycle badges live.
 *
 * @example
 * ```ts
 * const interval = DASHBOARD_STATUS_REFRESH_INTERVAL_MS; // 5000
 * ```
 */
export const DASHBOARD_STATUS_REFRESH_INTERVAL_MS = 5_000;

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
 * "danger" palette and surfaces the error text below the badge. Polling
 * defaults to {@link DASHBOARD_STATUS_REFRESH_INTERVAL_MS} so cockpit updates
 * feel live without overwhelming the backend.
 */
export default function DashboardStatusBadge({
  kind,
  name,
  refreshIntervalMs = DASHBOARD_STATUS_REFRESH_INTERVAL_MS,
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
