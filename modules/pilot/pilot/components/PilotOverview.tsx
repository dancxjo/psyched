import { useCallback, useMemo } from "preact/hooks";
import {
  type ConnectionStatus,
  useCockpitTopic,
} from "@pilot/lib/cockpit.ts";

import {
  Card,
  CONNECTION_STATUS_LABELS,
  Panel,
  toneFromConnection,
} from "./dashboard.tsx";

interface PilotStatus {
  timestamp?: string;
  activeModules?: number;
  note?: string;
}

interface PilotOverviewProps {
  version: string;
  description: string;
}

function formatTimestamp(timestamp: string | undefined) {
  if (!timestamp) return "No status yet";
  try {
    const parsed = new Date(timestamp);
    if (Number.isNaN(parsed.getTime())) {
      return timestamp;
    }
    return parsed.toLocaleString();
  } catch (_error) {
    return timestamp;
  }
}

export default function PilotOverview({
  version,
  description,
}: PilotOverviewProps) {
  const { status, data, error, publish } = useCockpitTopic<PilotStatus>(
    "/pilot/status",
    {
      replay: true,
      initialValue: {
        note: "Awaiting heartbeat from backend",
      },
    },
  );

  const connectionStatus = status as ConnectionStatus;
  const connectionLabel =
    CONNECTION_STATUS_LABELS[connectionStatus] ?? "Unknown";
  const formattedTimestamp = useMemo(
    () => formatTimestamp(data?.timestamp),
    [data?.timestamp],
  );

  const handlePing = useCallback(() => {
    if (typeof window === "undefined") {
      return;
    }

    publish({
      note: "Operator ping",
      timestamp: new Date().toISOString(),
    });
  }, [publish]);

  const activeModules = data?.activeModules;
  const activeModulesLabel =
    typeof activeModules === "number"
      ? `${activeModules} active`
      : "Active modules —";

  return (
    <Panel
      title="Pilot service"
      subtitle={description}
      accent="amber"
      badges={[
        {
          label: connectionLabel,
          tone: toneFromConnection(connectionStatus),
          pulse: connectionStatus === "connecting",
        },
        {
          label: activeModulesLabel,
          tone: typeof activeModules === "number" && activeModules > 0
            ? "info"
            : "neutral",
        },
      ]}
      actions={<span class="chip">v{version}</span>}
    >
      <div class="panel-grid panel-grid--stretch">
        <Card title="Bridge status" subtitle="Websocket health" tone="amber">
          <dl class="stat-list">
            <div class="stat-list__item">
              <dt>Connection</dt>
              <dd>{connectionLabel}</dd>
            </div>
            <div class="stat-list__item">
              <dt>Last heartbeat</dt>
              <dd>{formattedTimestamp}</dd>
            </div>
            <div class="stat-list__item">
              <dt>Active modules</dt>
              <dd>{typeof activeModules === "number" ? activeModules : "—"}</dd>
            </div>
            <div class="stat-list__item">
              <dt>Version</dt>
              <dd>{version}</dd>
            </div>
          </dl>
        </Card>

        <Card title="Status message" subtitle="Latest broadcast" tone="neutral">
          <p class="note">{data?.note ?? "No messages received yet."}</p>
          {error && <p class="note note--alert">{error}</p>}
        </Card>

        <Card title="Quick actions" subtitle="Connectivity checks" tone="teal">
          <p class="note">
            Broadcast a heartbeat to verify the cockpit bridge and frontend can
            exchange messages end-to-end.
          </p>
          <div class="button-group">
            <button
              class="button button--primary"
              type="button"
              onClick={handlePing}
            >
              Send ping
            </button>
          </div>
        </Card>
      </div>
    </Panel>
  );
}
