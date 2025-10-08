import {
  useCallback,
  useEffect,
  useMemo,
  useRef,
  useState,
} from "preact/hooks";
import { useCockpitTopic } from "@pilot/lib/cockpit.ts";

import {
  CONNECTION_STATUS_LABELS,
  Card,
  Panel,
  toneFromConnection,
} from "@pilot/components/dashboard.tsx";
import {
  formatNullableNumber,
  formatRelativeTime,
} from "../../../pilot/frontend/lib/format.ts";

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

  const telemetryBadge = {
    label: CONNECTION_STATUS_LABELS[telemetryStatus] ?? "Unknown",
    tone: toneFromConnection(telemetryStatus),
  };
  const cmdVelBadge = {
    label: CONNECTION_STATUS_LABELS[cmdVelStatus] ?? "Unknown",
    tone: toneFromConnection(cmdVelStatus),
  };

  const batteryPercentage = telemetry?.battery?.percentage ?? null;
  const batteryRatio = useMemo(() => {
    const ratio = telemetry?.battery?.charge_ratio;
    if (typeof ratio === "number" && Number.isFinite(ratio)) {
      return clamp(ratio, 0, 1);
    }
    if (
      typeof batteryPercentage === "number" &&
      Number.isFinite(batteryPercentage)
    ) {
      return clamp(batteryPercentage / 100, 0, 1);
    }
    return null;
  }, [telemetry?.battery?.charge_ratio, batteryPercentage]);

  const batteryStats = useMemo(
    () => [
      {
        label: "V",
        value: formatNullableNumber(telemetry?.battery?.voltage_v, {
          fractionDigits: 1,
        }),
      },
      {
        label: "A",
        value: formatNullableNumber(telemetry?.battery?.current_a, {
          fractionDigits: 2,
        }),
      },
      {
        label: "°C",
        value: formatNullableNumber(telemetry?.battery?.temperature_c, {
          fractionDigits: 0,
        }),
      },
    ],
    [
      telemetry?.battery?.voltage_v,
      telemetry?.battery?.current_a,
      telemetry?.battery?.temperature_c,
    ],
  );

  const modeLabel = telemetry?.status?.mode ?? "—";
  const chargingLabel = telemetry?.status?.charging_state ??
    telemetry?.battery?.charging_state ?? "—";
  const lastCommandLabel = formatRelativeTime(
    telemetry?.status?.last_command_ms,
  );
  const lastUpdateLabel = formatRelativeTime(telemetry?.last_update_ms);

  const commanded = telemetry?.motion?.command;
  const odometry = telemetry?.motion?.odometry;
  const hazards = telemetry?.hazards ?? {};

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

  const batteryPercentageLabel = batteryPercentage === null ||
      batteryPercentage === undefined
    ? "—"
    : `${Math.round(batteryPercentage)}%`;

  return (
    <Panel
      title="Create base"
      subtitle="Compact drive, power, and hazard console for Pete's drivetrain."
      accent="amber"
      badges={[telemetryBadge, cmdVelBadge]}
    >
      <div class="panel-grid panel-grid--stretch">
        <Card
          title="Power"
          tone="amber"
          icon={<BatteryIcon level={batteryRatio} />}
        >
          <div class="stat-metric">
            <span class="stat-metric__value">{batteryPercentageLabel}</span>
            <span class="stat-metric__caption">Charge</span>
          </div>
          <dl class="stat-list stat-list--columns">
            {batteryStats.map(({ label, value }) => (
              <div class="stat-list__item" key={label}>
                <dt>{label}</dt>
                <dd>{value}</dd>
              </div>
            ))}
          </dl>
          <p class="note">
            {chargingLabel}
            {telemetry?.battery?.is_charging ? " • charging" : ""}
          </p>
        </Card>

        <Card title="Status" tone="teal" icon={<StatusIcon />}>
          <dl class="stat-list">
            <div class="stat-list__item">
              <dt>Mode</dt>
              <dd>{modeLabel}</dd>
            </div>
            <div class="stat-list__item">
              <dt>Last command</dt>
              <dd>{lastCommandLabel}</dd>
            </div>
            <div class="stat-list__item">
              <dt>Telemetry</dt>
              <dd>{lastUpdateLabel}</dd>
            </div>
          </dl>
          {telemetryError && (
            <p class="note note--alert">{telemetryError}</p>
          )}
        </Card>

        <Card title="Motion" tone="cyan" icon={<MotionIcon />}>
          <div class="command-matrix">
            <div class="command-matrix__row">
              <span class="chip">cmd</span>
              <span>
                {formatNullableNumber(commanded?.linear_x, {
                  fractionDigits: 2,
                })} m/s
              </span>
              <span>
                {formatNullableNumber(commanded?.angular_z, {
                  fractionDigits: 2,
                })} rad/s
              </span>
            </div>
            <div class="command-matrix__row">
              <span class="chip">odom</span>
              <span>
                {formatNullableNumber(odometry?.linear_x, {
                  fractionDigits: 2,
                })} m/s
              </span>
              <span>
                {formatNullableNumber(odometry?.angular_z, {
                  fractionDigits: 2,
                })} rad/s
              </span>
            </div>
          </div>
        </Card>

        <Card title="Hazards" tone="magenta" icon={<HazardIcon />}>
          <div class="hazard-grid">
            <div>
              <h3 class="hazard-grid__title">Bumpers</h3>
              <div class="indicator-dots">
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
            <div>
              <h3 class="hazard-grid__title">Cliffs</h3>
              <div class="indicator-dots">
                <span
                  class={hazardClass(hazards.cliff_left)}
                  title="Left cliff"
                >
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
                <span
                  class={hazardClass(hazards.cliff_right)}
                  title="Right cliff"
                >
                  R
                </span>
              </div>
            </div>
          </div>
        </Card>
      </div>

      <Card
        title="Manual control"
        tone="teal"
        icon={<JoystickIcon />}
        footer={
          <dl class="stat-list stat-list--columns">
            <div class="stat-list__item">
              <dt>linear.x</dt>
              <dd>{currentTwist.linear.x.toFixed(2)} m/s</dd>
            </div>
            <div class="stat-list__item">
              <dt>angular.z</dt>
              <dd>{currentTwist.angular.z.toFixed(2)} rad/s</dd>
            </div>
          </dl>
        }
        actions={
          <button
            type="button"
            class="control-button"
            onClick={resetStick}
            title="Stop"
          >
            <StopIcon />
            <span class="sr-only">Stop</span>
          </button>
        }
      >
        <p class="note">
          Drag the pad to command velocities (±{MAX_LINEAR_X.toFixed(2)}{" "}
          m/s, ±{MAX_ANGULAR_Z.toFixed(2)} rad/s). Release to halt.
        </p>
        <div
          ref={joystickRef}
          class={`joystick${isDragging ? " joystick--active" : ""}`}
          aria-label="Drive joystick"
          role="application"
        >
          <div
            class={`joystick__thumb${
              isDragging ? " joystick__thumb--active" : ""
            }`}
            style={joystickThumbStyle}
          />
          <div
            class="joystick__axis joystick__axis--x"
            aria-hidden="true"
          />
          <div
            class="joystick__axis joystick__axis--y"
            aria-hidden="true"
          />
        </div>
      </Card>
    </Panel>
  );
}

function BatteryIcon({ level }: { level: number | null }) {
  const clamped = level === null ? 0 : Math.min(1, Math.max(0, level));
  const width = 36 * clamped;
  return (
    <svg viewBox="0 0 48 24" class="icon" aria-hidden="true">
      <rect
        x="1"
        y="4"
        width="40"
        height="16"
        rx="3"
        class="icon__outline"
      />
      <rect
        x="42"
        y="9"
        width="5"
        height="6"
        rx="1"
        class="icon__outline"
      />
      <rect
        x="3"
        y="6"
        width={width}
        height="12"
        rx="2"
        class="icon__fill"
      />
    </svg>
  );
}

function StatusIcon() {
  return (
    <svg viewBox="0 0 24 24" class="icon" aria-hidden="true">
      <circle cx="12" cy="12" r="8" class="icon__outline" />
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
    <svg viewBox="0 0 24 24" class="icon" aria-hidden="true">
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
    <svg viewBox="0 0 24 24" class="icon" aria-hidden="true">
      <path
        d="M12 3 2.5 20.5h19L12 3z"
        fill="none"
        stroke="currentColor"
        stroke-width="2"
        stroke-linejoin="round"
      />
      <circle cx="12" cy="16" r="1.5" class="icon__fill" />
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
    <svg viewBox="0 0 24 24" class="icon" aria-hidden="true">
      <circle cx="12" cy="12" r="3" class="icon__fill" />
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
    <svg viewBox="0 0 16 16" class="icon" aria-hidden="true">
      <rect x="3" y="3" width="10" height="10" rx="2" />
    </svg>
  );
}

function hazardClass(active?: boolean) {
  return `indicator-dot${active ? " indicator-dot--active" : ""}`;
}
