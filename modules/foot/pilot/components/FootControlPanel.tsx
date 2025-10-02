// @ts-nocheck
import {
  useCallback,
  useEffect,
  useMemo,
  useRef,
  useState,
} from "preact/hooks";
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

type Vector3 = {
  x: number;
  y: number;
  z: number;
};

type Twist = {
  linear: Vector3;
  angular: Vector3;
};

const ZERO_TWIST: Twist = {
  linear: { x: 0, y: 0, z: 0 },
  angular: { x: 0, y: 0, z: 0 },
};

const MAX_LINEAR_X = 0.45; // m/s
const MAX_ANGULAR_Z = 1.2; // rad/s

const clamp = (value: number, min = -1, max = 1) =>
  Math.min(max, Math.max(min, value));

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

  const {
    status: cmdVelStatus,
    data: currentTwist = ZERO_TWIST,
    publish: publishCmdVel,
  } = useCockpitTopic<Twist>("/cmd_vel", {
    replay: true,
    initialValue: ZERO_TWIST,
  });

  const connectionLabel = STATUS_LABELS[status] ?? "Unknown";
  const badgeVariant = STATUS_LABELS[status] ? status : "idle";
  const cmdVelLabel = STATUS_LABELS[cmdVelStatus] ?? "Unknown";
  const cmdVelBadgeVariant = STATUS_LABELS[cmdVelStatus]
    ? cmdVelStatus
    : "idle";

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

  type StickPosition = { x: number; y: number };

  const joystickRef = useRef<HTMLDivElement | null>(null);
  const [stickPosition, setStickPosition] = useState<StickPosition>({
    x: 0,
    y: 0,
  });
  const [isDragging, setIsDragging] = useState(false);
  const activePointerId = useRef<number | null>(null);

  const sendTwist = useCallback(
    (normX: number, normY: number) => {
      const linearX = clamp(-normY, -1, 1) * MAX_LINEAR_X;
      const angularZ = clamp(normX, -1, 1) * MAX_ANGULAR_Z;
      const roundedLinearX = Number(linearX.toFixed(3));
      const roundedAngularZ = Number(angularZ.toFixed(3));

      publishCmdVel({
        linear: { x: roundedLinearX, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: roundedAngularZ },
      });
    },
    [publishCmdVel],
  );

  const updateStickFromPointer = useCallback(
    (clientX: number, clientY: number) => {
      const pad = joystickRef.current;
      if (!pad) return;

      const rect = pad.getBoundingClientRect();
      const rawX = (clientX - rect.left) / rect.width;
      const rawY = (clientY - rect.top) / rect.height;

      let normX = rawX * 2 - 1;
      let normY = rawY * 2 - 1;

      const magnitude = Math.hypot(normX, normY);
      if (magnitude > 1) {
        normX /= magnitude;
        normY /= magnitude;
      }

      setStickPosition({ x: normX, y: normY });
      sendTwist(normX, normY);
    },
    [sendTwist],
  );

  const resetStick = useCallback(() => {
    setStickPosition({ x: 0, y: 0 });
    sendTwist(0, 0);
  }, [sendTwist]);

  const handlePointerDown = useCallback(
    (event: PointerEvent) => {
      const pad = joystickRef.current;
      if (!pad) return;

      pad.setPointerCapture(event.pointerId);
      activePointerId.current = event.pointerId;
      setIsDragging(true);
      updateStickFromPointer(event.clientX, event.clientY);
    },
    [updateStickFromPointer],
  );

  const handlePointerMove = useCallback(
    (event: PointerEvent) => {
      if (!isDragging || activePointerId.current !== event.pointerId) return;
      updateStickFromPointer(event.clientX, event.clientY);
    },
    [isDragging, updateStickFromPointer],
  );

  const handlePointerUp = useCallback(
    (event: PointerEvent) => {
      if (activePointerId.current !== event.pointerId) return;
      const pad = joystickRef.current;
      if (pad?.hasPointerCapture(event.pointerId)) {
        pad.releasePointerCapture(event.pointerId);
      }
      activePointerId.current = null;
      setIsDragging(false);
      resetStick();
    },
    [resetStick],
  );

  useEffect(() => {
    const pad = joystickRef.current;
    if (!pad) return;

    const onPointerDown = (event: PointerEvent) => handlePointerDown(event);
    const onPointerMove = (event: PointerEvent) => handlePointerMove(event);
    const onPointerUp = (event: PointerEvent) => handlePointerUp(event);
    const onPointerCancel = (event: PointerEvent) => handlePointerUp(event);

    pad.addEventListener("pointerdown", onPointerDown);
    pad.addEventListener("pointermove", onPointerMove);
    pad.addEventListener("pointerup", onPointerUp);
    pad.addEventListener("pointercancel", onPointerCancel);

    return () => {
      pad.removeEventListener("pointerdown", onPointerDown);
      pad.removeEventListener("pointermove", onPointerMove);
      pad.removeEventListener("pointerup", onPointerUp);
      pad.removeEventListener("pointercancel", onPointerCancel);
    };
  }, [handlePointerDown, handlePointerMove, handlePointerUp]);

  useEffect(() => {
    if (isDragging) return;

    const { linear, angular } = currentTwist;
    const derivedX = clamp(angular.z / MAX_ANGULAR_Z);
    const derivedY = clamp(-linear.x / MAX_LINEAR_X);

    setStickPosition((prev) => {
      if (
        Math.abs(prev.x - derivedX) < 0.01 && Math.abs(prev.y - derivedY) < 0.01
      ) {
        return prev;
      }
      return { x: derivedX, y: derivedY };
    });
  }, [currentTwist, isDragging]);

  useEffect(() => {
    return () => {
      publishCmdVel(ZERO_TWIST);
    };
  }, [publishCmdVel]);

  const joystickThumbStyle = useMemo(
    () => ({
      "--stick-x": String(stickPosition.x),
      "--stick-y": String(stickPosition.y),
    }),
    [stickPosition],
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

      <section class="foot-panel__joystick">
        <header class="foot-panel__joystick-header">
          <h2>Joystick</h2>
          <span
            class={`foot-panel__badge foot-panel__badge--${cmdVelBadgeVariant}`}
          >
            {cmdVelLabel}
          </span>
        </header>
        <p class="foot-panel__joystick-description">
          Drag inside the pad to send velocity commands on{" "}
          <code>/cmd_vel</code>. Vertical motion maps to forward speed
          (±{MAX_LINEAR_X.toFixed(2)}{" "}
          m/s). Horizontal motion adjusts angular velocity (±
          {MAX_ANGULAR_Z.toFixed(1)} rad/s). Releasing the stick sends a stop.
        </p>
        <div
          ref={joystickRef}
          class={`foot-panel__joystick-pad${
            isDragging ? " foot-panel__joystick-pad--active" : ""
          }`}
          aria-label="Drive joystick"
          role="application"
        >
          <div
            class={`foot-panel__joystick-thumb${
              isDragging ? " foot-panel__joystick-thumb--active" : ""
            }`}
            style={joystickThumbStyle}
          />
          <div
            class="foot-panel__joystick-axis foot-panel__joystick-axis--x"
            aria-hidden="true"
          />
          <div
            class="foot-panel__joystick-axis foot-panel__joystick-axis--y"
            aria-hidden="true"
          />
        </div>
        <dl class="foot-panel__joystick-readout">
          <div>
            <dt>linear.x</dt>
            <dd>
              {currentTwist.linear.x.toFixed(2)} m/s
            </dd>
          </div>
          <div>
            <dt>angular.z</dt>
            <dd>
              {currentTwist.angular.z.toFixed(2)} rad/s
            </dd>
          </div>
        </dl>
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
