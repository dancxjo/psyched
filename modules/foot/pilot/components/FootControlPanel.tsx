import {
  useCallback,
  useEffect,
  useMemo,
  useRef,
  useState,
} from "preact/hooks";
import { type ConnectionStatus, useCockpitTopic } from "@pilot/lib/cockpit.ts";

type Vector3 = { x: number; y: number; z: number };

type TwistPayload = {
  linear: Vector3;
  angular: Vector3;
};

type TwistSummary = {
  linear_x: number;
  linear_y: number;
  linear_z: number;
  angular_x: number;
  angular_y: number;
  angular_z: number;
};

type FootTelemetrySnapshot = {
  battery?: {
    percentage?: number;
    charge_ratio?: number;
    charge_ah?: number;
    capacity_ah?: number;
    current_a?: number;
    voltage_v?: number;
    temperature_c?: number;
    charging_state?: string;
    is_charging?: boolean;
  };
  status?: {
    mode?: string;
    mode_code?: number;
    charging_state?: string;
    charging_state_code?: number;
    last_command_ms?: number;
  };
  motion?: {
    command?: TwistSummary;
    odometry?: TwistSummary;
  };
  hazards?: {
    bumper_left?: boolean;
    bumper_right?: boolean;
    bumper_light_front?: boolean;
    bumper_light_center?: boolean;
    cliff_left?: boolean;
    cliff_front_left?: boolean;
    cliff_right?: boolean;
    cliff_front_right?: boolean;
  };
  last_update_ms?: number;
};

const STATUS_LABELS: Record<ConnectionStatus, string> = {
  idle: "Idle",
  connecting: "Connecting",
  open: "Connected",
  closed: "Disconnected",
  error: "Error",
};

const ZERO_TWIST: TwistPayload = {
  linear: { x: 0, y: 0, z: 0 },
  angular: { x: 0, y: 0, z: 0 },
};

const MAX_LINEAR_X = 0.5; // m/s
const MAX_ANGULAR_Z = 4.25; // rad/s

const clamp = (value: number, min = -1, max = 1) =>
  Math.min(max, Math.max(min, value));

export default function FootControlPanel() {
  const {
    status: telemetryStatus,
    data: telemetry,
    error: telemetryError,
  } = useCockpitTopic<FootTelemetrySnapshot>("/foot/telemetry", {
    replay: true,
    initialValue: {},
  });

  const {
    status: cmdVelStatus,
    data: currentTwist = ZERO_TWIST,
    publish: publishCmdVel,
  } = useCockpitTopic<TwistPayload>("/cmd_vel", {
    replay: true,
    initialValue: ZERO_TWIST,
  });

  const connectionLabel = STATUS_LABELS[telemetryStatus] ?? "Unknown";
  const telemetryBadgeVariant = STATUS_LABELS[telemetryStatus]
    ? telemetryStatus
    : "idle";
  const cmdVelLabel = STATUS_LABELS[cmdVelStatus] ?? "Unknown";
  const cmdVelBadgeVariant = STATUS_LABELS[cmdVelStatus] ? cmdVelStatus : "idle";

  const batteryPercentage = telemetry?.battery?.percentage ?? null;
  const batteryRatio = useMemo(() => {
    const ratio = telemetry?.battery?.charge_ratio;
    if (typeof ratio === "number" && Number.isFinite(ratio)) {
      return clamp(ratio, 0, 1);
    }
    if (typeof batteryPercentage === "number" && Number.isFinite(batteryPercentage)) {
      return clamp(batteryPercentage / 100, 0, 1);
    }
    return null;
  }, [telemetry?.battery?.charge_ratio, batteryPercentage]);

  const batteryStats = useMemo(
    () => [
      { label: "V", value: formatNumber(telemetry?.battery?.voltage_v, 1) },
      { label: "A", value: formatNumber(telemetry?.battery?.current_a, 2) },
      {
        label: "°C",
        value: formatNumber(telemetry?.battery?.temperature_c, 0),
      },
    ],
    [telemetry?.battery?.voltage_v, telemetry?.battery?.current_a, telemetry?.battery?.temperature_c],
  );

  const modeLabel = telemetry?.status?.mode ?? "—";
  const chargingLabel =
    telemetry?.status?.charging_state ?? telemetry?.battery?.charging_state ?? "—";
  const lastCommandLabel = formatRelativeMillis(
    telemetry?.status?.last_command_ms,
  );
  const lastUpdateLabel = formatRelativeMillis(telemetry?.last_update_ms);

  const commanded = telemetry?.motion?.command;
  const odometry = telemetry?.motion?.odometry;
  const hazards = telemetry?.hazards ?? {};

  type StickPosition = { x: number; y: number };
  const joystickRef = useRef<HTMLDivElement | null>(null);
  const [stickPosition, setStickPosition] = useState<StickPosition>({ x: 0, y: 0 });
  const [isDragging, setIsDragging] = useState(false);
  const activePointerId = useRef<number | null>(null);

  const sendTwist = useCallback(
    (normX: number, normY: number) => {
      const linearX = clamp(-normY, -1, 1) * MAX_LINEAR_X;
      const angularZ = clamp(normX, -1, 1) * MAX_ANGULAR_Z;
      publishCmdVel({
        linear: {
          x: Number(linearX.toFixed(3)),
          y: 0,
          z: 0,
        },
        angular: {
          x: 0,
          y: 0,
          z: Number(angularZ.toFixed(3)),
        },
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
        Math.abs(prev.x - derivedX) < 0.01 &&
        Math.abs(prev.y - derivedY) < 0.01
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
    <article class="foot-panel foot-panel--lcars">
      <header class="foot-panel__header">
        <div class="foot-panel__heading">
          <span class="foot-panel__heading-accent" aria-hidden="true" />
          <div>
            <h1 class="foot-panel__title">Create base</h1>
            <p class="foot-panel__description">
              Compact drive, power, and hazard console for Pete&apos;s drivetrain.
            </p>
          </div>
        </div>
        <div class="foot-panel__status-chips">
          <span class={`foot-panel__badge foot-panel__badge--${telemetryBadgeVariant}`}>
            {connectionLabel}
          </span>
          <span class={`foot-panel__badge foot-panel__badge--${cmdVelBadgeVariant}`}>
            {cmdVelLabel}
          </span>
        </div>
      </header>

      <section class="foot-panel__grid">
        <div class="foot-panel__tile foot-panel__tile--battery">
          <div class="foot-panel__tile-header">
            <BatteryIcon level={batteryRatio} />
            <span>Power</span>
          </div>
          <div class="foot-panel__tile-body">
            <strong class="foot-panel__tile-value">
              {batteryPercentage === null || batteryPercentage === undefined
                ? "—"
                : `${batteryPercentage.toFixed(0)}%`}
            </strong>
            <ul class="foot-panel__mini-list">
              {batteryStats.map(({ label, value }) => (
                <li key={label}>
                  <span>{label}</span>
                  <strong>{value}</strong>
                </li>
              ))}
            </ul>
            <p class="foot-panel__tile-meta">
              {chargingLabel}
              {telemetry?.battery?.is_charging ? " • charging" : ""}
            </p>
          </div>
        </div>

        <div class="foot-panel__tile foot-panel__tile--status">
          <div class="foot-panel__tile-header">
            <StatusIcon />
            <span>Status</span>
          </div>
          <div class="foot-panel__tile-body">
            <div class="foot-panel__chip-row">
              <span class="foot-panel__chip">Mode</span>
              <strong>{modeLabel}</strong>
            </div>
            <div class="foot-panel__chip-row">
              <span class="foot-panel__chip">Cmd</span>
              <strong>{lastCommandLabel}</strong>
            </div>
            <div class="foot-panel__chip-row">
              <span class="foot-panel__chip">Telemetry</span>
              <strong>{lastUpdateLabel}</strong>
            </div>
            {telemetryError && (
              <p class="foot-panel__status-note">{telemetryError}</p>
            )}
          </div>
        </div>

        <div class="foot-panel__tile foot-panel__tile--motion">
          <div class="foot-panel__tile-header">
            <MotionIcon />
            <span>Motion</span>
          </div>
          <div class="foot-panel__tile-body foot-panel__motion">
            <div class="foot-panel__motion-row">
              <span class="foot-panel__chip">cmd</span>
              <span>{formatNumber(commanded?.linear_x, 2)} m/s</span>
              <span>{formatNumber(commanded?.angular_z, 2)} rad/s</span>
            </div>
            <div class="foot-panel__motion-row">
              <span class="foot-panel__chip">odom</span>
              <span>{formatNumber(odometry?.linear_x, 2)} m/s</span>
              <span>{formatNumber(odometry?.angular_z, 2)} rad/s</span>
            </div>
          </div>
        </div>

        <div class="foot-panel__tile foot-panel__tile--hazards">
          <div class="foot-panel__tile-header">
            <HazardIcon />
            <span>Hazards</span>
          </div>
          <div class="foot-panel__tile-body foot-panel__hazards">
            <div class="foot-panel__hazard-group">
              <h3>Bumpers</h3>
              <div class="foot-panel__hazard-dots">
                <span
                  class={hazardClass(hazards.bumper_left)}
                  title="Left bumper"
                >
                  L
                </span>
                <span
                  class={hazardClass(hazards.bumper_right)}
                  title="Right bumper"
                >
                  R
                </span>
                <span
                  class={hazardClass(hazards.bumper_light_front)}
                  title="Front IR"
                >
                  F
                </span>
                <span
                  class={hazardClass(hazards.bumper_light_center)}
                  title="Center IR"
                >
                  C
                </span>
              </div>
            </div>
            <div class="foot-panel__hazard-group">
              <h3>Cliffs</h3>
              <div class="foot-panel__hazard-dots">
                <span class={hazardClass(hazards.cliff_left)} title="Left cliff">
                  L
                </span>
                <span
                  class={hazardClass(hazards.cliff_front_left)}
                  title="Front-left cliff"
                >
                  FL
                </span>
                <span
                  class={hazardClass(hazards.cliff_front_right)}
                  title="Front-right cliff"
                >
                  FR
                </span>
                <span class={hazardClass(hazards.cliff_right)} title="Right cliff">
                  R
                </span>
              </div>
            </div>
          </div>
        </div>
      </section>

      <section class="foot-panel__joystick">
        <header class="foot-panel__joystick-header">
          <div class="foot-panel__tile-header">
            <JoystickIcon />
            <span>Manual control</span>
          </div>
          <button
            type="button"
            class="foot-panel__chip foot-panel__chip--action"
            onClick={resetStick}
            title="Stop"
          >
            <StopIcon />
            <span class="sr-only">Stop</span>
          </button>
        </header>
        <p class="foot-panel__joystick-description">
          Drag the pad to command velocities (±{MAX_LINEAR_X.toFixed(2)} m/s,
          ±{MAX_ANGULAR_Z.toFixed(2)} rad/s). Release to halt.
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
            <dd>{currentTwist.linear.x.toFixed(2)} m/s</dd>
          </div>
          <div>
            <dt>angular.z</dt>
            <dd>{currentTwist.angular.z.toFixed(2)} rad/s</dd>
          </div>
        </dl>
      </section>
    </article>
  );
}

function BatteryIcon({ level }: { level: number | null }) {
  const clamped = level === null ? 0 : Math.min(1, Math.max(0, level));
  const width = 36 * clamped;
  return (
    <svg viewBox="0 0 48 24" class="foot-panel__icon" aria-hidden="true">
      <rect
        x="1"
        y="4"
        width="40"
        height="16"
        rx="3"
        class="foot-panel__icon-outline"
      />
      <rect
        x="42"
        y="9"
        width="5"
        height="6"
        rx="1"
        class="foot-panel__icon-outline"
      />
      <rect
        x="3"
        y="6"
        width={width}
        height="12"
        rx="2"
        class="foot-panel__icon-fill"
      />
    </svg>
  );
}

function StatusIcon() {
  return (
    <svg viewBox="0 0 24 24" class="foot-panel__icon" aria-hidden="true">
      <circle cx="12" cy="12" r="8" class="foot-panel__icon-outline" />
      <path
        d="M12 6v6l4 2"
        fill="none"
        stroke="currentColor"
        stroke-width="2"
        stroke-linecap="round"
        stroke-linejoin="round"
      />
    </svg>
  );
}

function MotionIcon() {
  return (
    <svg viewBox="0 0 24 24" class="foot-panel__icon" aria-hidden="true">
      <path
        d="M4 12h16M12 4v16M16 8l4 4-4 4M8 16l-4-4 4-4"
        fill="none"
        stroke="currentColor"
        stroke-width="2"
        stroke-linecap="round"
        stroke-linejoin="round"
      />
    </svg>
  );
}

function HazardIcon() {
  return (
    <svg viewBox="0 0 24 24" class="foot-panel__icon" aria-hidden="true">
      <path
        d="M12 3 2.5 20.5h19L12 3z"
        fill="none"
        stroke="currentColor"
        stroke-width="2"
        stroke-linejoin="round"
      />
      <circle cx="12" cy="16" r="1.5" class="foot-panel__icon-fill" />
      <path
        d="M12 9v5"
        stroke="currentColor"
        stroke-width="2"
        stroke-linecap="round"
      />
    </svg>
  );
}

function JoystickIcon() {
  return (
    <svg viewBox="0 0 24 24" class="foot-panel__icon" aria-hidden="true">
      <circle cx="12" cy="12" r="3" class="foot-panel__icon-fill" />
      <path
        d="M12 3v6M12 15v6M3 12h6M15 12h6"
        stroke="currentColor"
        stroke-width="2"
        stroke-linecap="round"
      />
    </svg>
  );
}

function StopIcon() {
  return (
    <svg viewBox="0 0 16 16" class="foot-panel__icon" aria-hidden="true">
      <rect x="3" y="3" width="10" height="10" rx="2" />
    </svg>
  );
}

function formatNumber(value?: number | null, fractionDigits = 2) {
  if (value === undefined || value === null || Number.isNaN(value)) {
    return "—";
  }
  return value.toFixed(fractionDigits);
}

function formatRelativeMillis(value?: number) {
  if (!value) return "—";
  const delta = Date.now() - value;
  if (delta < 0) return "0s";
  const seconds = Math.floor(delta / 1000);
  if (seconds < 1) return "<1s";
  if (seconds < 60) return `${seconds}s`;
  const minutes = Math.floor(seconds / 60);
  if (minutes < 60) return `${minutes}m`;
  const hours = Math.floor(minutes / 60);
  if (hours < 24) return `${hours}h`;
  const days = Math.floor(hours / 24);
  return `${days}d`;
}

function hazardClass(active?: boolean) {
  return `foot-panel__hazard-dot${active ? " foot-panel__hazard-dot--active" : ""}`;
}
