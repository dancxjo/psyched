import { useCallback, useMemo } from "preact/hooks";
import {
  type ConnectionStatus,
  useCockpitTopic,
} from "../../frontend/lib/cockpit.ts";

interface PilotStatus {
  timestamp?: string;
  activeModules?: number;
  note?: string;
}

interface PilotOverviewProps {
  version: string;
  description: string;
}

const STATUS_LABELS: Record<ConnectionStatus, string> = {
  idle: "Idle",
  connecting: "Connecting",
  open: "Connected",
  closed: "Disconnected",
  error: "Error",
};

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

  const formattedTimestamp = useMemo(() => {
    const source = data?.timestamp;
    if (!source) {
      return "No status yet";
    }

    try {
      const parsed = new Date(source);
      if (Number.isNaN(parsed.getTime())) {
        return source;
      }
      return parsed.toLocaleString();
    } catch (_err) {
      return source;
    }
  }, [data?.timestamp]);

  const handlePing = useCallback(() => {
    if (typeof window === "undefined") {
      return;
    }

    publish({
      note: "Operator ping",
      timestamp: new Date().toISOString(),
    });
  }, [publish]);

  const activeModules = data?.activeModules ?? "—";
  const connectionLabel = STATUS_LABELS[connectionStatus];

  return (
    <article class="pilot-overview">
      <header class="pilot-overview__header">
        <div>
          <h1 class="pilot-overview__title">Pilot service</h1>
          <p class="pilot-overview__description">{description}</p>
        </div>
        <div
          class={`pilot-overview__badge pilot-overview__badge--${connectionStatus}`}
        >
          {connectionLabel}
        </div>
      </header>

      <dl class="pilot-overview__metrics">
        <div>
          <dt>Version</dt>
          <dd>{version}</dd>
        </div>
        <div>
          <dt>Connection status</dt>
          <dd>{connectionLabel}</dd>
        </div>
        <div>
          <dt>Last heartbeat</dt>
          <dd>{formattedTimestamp}</dd>
        </div>
        <div>
          <dt>Active modules</dt>
          <dd>{activeModules}</dd>
        </div>
      </dl>

      <section class="pilot-overview__actions">
        <div>
          <h2>Quick actions</h2>
          <p>
            Broadcast a heartbeat to verify websocket connectivity. Pings are
            no-ops but show monitoring pipelines end-to-end.
          </p>
          <button
            class="pilot-overview__button"
            type="button"
            onClick={handlePing}
          >
            Send ping
          </button>
        </div>
        <div class="pilot-overview__status">
          <h3>Status message</h3>
          <p>{data?.note ?? "No messages received yet."}</p>
          {error && <p class="pilot-overview__error">{error}</p>}
        </div>
      </section>
    </article>
  );
}
