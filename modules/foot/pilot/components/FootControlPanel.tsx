import { useCallback, useMemo } from "preact/hooks";
import { type ConnectionStatus, useCockpitTopic } from "@pilot/lib/cockpit.ts";

type FootStatus = {
  batteryPct?: number | null;
  docked?: boolean;
  mode?: string;
  lastCommand?: string;
  lastUpdate?: string;
  faults?: string[];
};

const STATUS_LABELS: Record<ConnectionStatus, string> = {
  idle: "Idle",
  connecting: "Connecting",
  open: "Connected",
  closed: "Disconnected",
  error: "Error",
};

const DEFAULT_STATUS: FootStatus = {
  batteryPct: null,
  docked: false,
  mode: "standby",
  lastCommand: "—",
  lastUpdate: undefined,
  faults: [],
};

export default function FootControlPanel() {
  const {
    status,
    data: statusPayload = DEFAULT_STATUS,
    error,
    publish,
  } = useCockpitTopic<FootStatus>("/foot/status", {
    replay: true,
    initialValue: DEFAULT_STATUS,
  });

  const connectionLabel = STATUS_LABELS[status] ?? "Unknown";
  const badgeVariant = STATUS_LABELS[status] ? status : "idle";

  const formattedBattery = useMemo(() => {
    const pct = statusPayload.batteryPct;
    if (pct === undefined || pct === null) return "—";
    return `${pct.toFixed(0)}%`;
  }, [statusPayload.batteryPct]);

  const formattedTimestamp = useMemo(() => {
    const timestamp = statusPayload.lastUpdate;
    if (!timestamp) return "Awaiting telemetry";
    const date = new Date(timestamp);
    if (Number.isNaN(date.getTime())) return timestamp;
    return date.toLocaleTimeString();
  }, [statusPayload.lastUpdate]);

  const faults = statusPayload.faults ?? [];

  const handleCommand = useCallback(
    (command: string) => () => {
      publish({
        ...statusPayload,
        lastCommand: command,
        lastUpdate: new Date().toISOString(),
      });
    },
    [publish, statusPayload],
  );

  return (
    <article class="foot-panel">
      <header class="foot-panel__header">
        <div>
          <h1 class="foot-panel__title">Drive base</h1>
          <p class="foot-panel__description">
            Monitor the Create robot state and dispatch high level commands.
          </p>
        </div>
        <span class={`foot-panel__badge foot-panel__badge--${badgeVariant}`}>
          {connectionLabel}
        </span>
      </header>

      <section class="foot-panel__metrics">
        <div>
          <h2>Status</h2>
          <dl>
            <div>
              <dt>Battery</dt>
              <dd>{formattedBattery}</dd>
            </div>
            <div>
              <dt>Mode</dt>
              <dd>{statusPayload.mode ?? "—"}</dd>
            </div>
            <div>
              <dt>Docked</dt>
              <dd>{statusPayload.docked ? "Yes" : "No"}</dd>
            </div>
            <div>
              <dt>Last update</dt>
              <dd>{formattedTimestamp}</dd>
            </div>
            <div>
              <dt>Last command</dt>
              <dd>{statusPayload.lastCommand ?? "—"}</dd>
            </div>
          </dl>
        </div>

        <div>
          <h2>Faults</h2>
          {faults.length === 0
            ? (
              <p class="foot-panel__fault foot-panel__fault--empty">
                No reported faults
              </p>
            )
            : (
              <ul class="foot-panel__fault-list">
                {faults.map((fault) => <li key={fault}>{fault}</li>)}
              </ul>
            )}
          {error && <p class="foot-panel__error">{error}</p>}
        </div>
      </section>

      <section class="foot-panel__actions">
        <h2>Manual commands</h2>
        <p>
          Dispatch simple motion cues while testing drivetrain integration.
          These commands mirror the `/foot/status` heartbeat so the backend can
          echo the most recent operator intent.
        </p>
        <div class="foot-panel__buttons">
          <button type="button" onClick={handleCommand("forward")}>
            Forward
          </button>
          <button type="button" onClick={handleCommand("stop")}>Stop</button>
          <button type="button" onClick={handleCommand("dock")}>Dock</button>
        </div>
      </section>
    </article>
  );
}
