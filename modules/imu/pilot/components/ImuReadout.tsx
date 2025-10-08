import { useMemo } from "preact/hooks";

import {
  CONNECTION_STATUS_LABELS,
  LcarsCard,
  LcarsPanel,
  toneFromConnection,
} from "../../../pilot/frontend/components/lcars.tsx";
import {
  formatNullableNumber,
  formatRelativeTime,
} from "../../../pilot/frontend/lib/format.ts";
import type { ConnectionStatus } from "@pilot/lib/cockpit.ts";

type Vector3 = {
  x?: number | null;
  y?: number | null;
  z?: number | null;
};

type OrientationEuler = {
  roll?: number | null;
  pitch?: number | null;
  yaw?: number | null;
};

type OrientationQuaternion = [number, number, number, number];

type Orientation = {
  euler?: OrientationEuler;
  quaternion?: OrientationQuaternion;
};

export type ImuSample = {
  frameId?: string;
  orientation?: Orientation;
  angularVelocity?: Vector3;
  linearAcceleration?: Vector3;
  temperatureC?: number | null;
  status?: string;
  lastUpdate?: Date | string | null;
};

export type ImuReadoutProps = {
  title?: string;
  sensor: ImuSample;
  connectionStatus?: ConnectionStatus;
};

const formatNumber = (value?: number | null, digits = 3) => {
  if (value === undefined || value === null || Number.isNaN(value)) {
    return "—";
  }
  return value.toFixed(digits);
};

const formatQuaternion = (quaternion?: OrientationQuaternion) => {
  if (!quaternion || quaternion.some((value) => Number.isNaN(value))) {
    return "—";
  }
  return quaternion.map((value) => value.toFixed(3)).join(", ");
};

export function ImuReadout({
  title = "IMU Sensor",
  sensor,
  connectionStatus,
}: ImuReadoutProps) {
  const euler = sensor.orientation?.euler;
  const quaternion = sensor.orientation?.quaternion;

  const connectionLabel =
    CONNECTION_STATUS_LABELS[connectionStatus ?? "idle"] ??
      "Unknown";
  const lastUpdate = useMemo(() => {
    const value = sensor.lastUpdate;
    if (!value) return undefined;
    if (value instanceof Date) {
      const time = value.getTime();
      return Number.isNaN(time) ? undefined : time;
    }
    const parsed = Date.parse(String(value));
    return Number.isNaN(parsed) ? undefined : parsed;
  }, [sensor.lastUpdate]);

  const eulerRows = useMemo(() => {
    if (!euler) return [];
    return (
      [
        { label: "Roll", value: formatNumber(euler.roll) },
        { label: "Pitch", value: formatNumber(euler.pitch) },
        { label: "Yaw", value: formatNumber(euler.yaw) },
      ] as const
    ).filter((row) => row.value !== "—");
  }, [euler?.roll, euler?.pitch, euler?.yaw]);

  const angularVelocityRows = useMemo(() => {
    const vector = sensor.angularVelocity;
    if (!vector) return [];
    return (
      [
        { label: "X", value: formatNumber(vector.x) },
        { label: "Y", value: formatNumber(vector.y) },
        { label: "Z", value: formatNumber(vector.z) },
      ] as const
    );
  }, [
    sensor.angularVelocity?.x,
    sensor.angularVelocity?.y,
    sensor.angularVelocity?.z,
  ]);

  const linearAccelerationRows = useMemo(() => {
    const vector = sensor.linearAcceleration;
    if (!vector) return [];
    return (
      [
        { label: "X", value: formatNumber(vector.x) },
        { label: "Y", value: formatNumber(vector.y) },
        { label: "Z", value: formatNumber(vector.z) },
      ] as const
    );
  }, [
    sensor.linearAcceleration?.x,
    sensor.linearAcceleration?.y,
    sensor.linearAcceleration?.z,
  ]);

  return (
    <LcarsPanel
      title={title}
      subtitle="Live inertial telemetry"
      accent="magenta"
      badges={[{
        label: connectionLabel,
        tone: toneFromConnection(connectionStatus),
      }]}
    >
      <div class="lcars-grid">
        <LcarsCard title="Identity" tone="magenta">
          <dl class="lcars-list">
            <div class="lcars-list__item">
              <dt>Frame</dt>
              <dd>{sensor.frameId ?? "—"}</dd>
            </div>
            <div class="lcars-list__item">
              <dt>Updated</dt>
              <dd>{formatRelativeTime(lastUpdate)}</dd>
            </div>
            {sensor.temperatureC !== undefined &&
              sensor.temperatureC !== null && (
              <div class="lcars-list__item">
                <dt>Temperature</dt>
                <dd>
                  {formatNullableNumber(sensor.temperatureC, {
                    fractionDigits: 1,
                  })} °C
                </dd>
              </div>
            )}
          </dl>
          {sensor.status && <p class="lcars-note">{sensor.status}</p>}
        </LcarsCard>

        {eulerRows.length > 0 && (
          <LcarsCard title="Euler" subtitle="degrees" tone="violet">
            <dl class="lcars-list lcars-list--columns">
              {eulerRows.map(({ label, value }) => (
                <div class="lcars-list__item" key={`euler-${label}`}>
                  <dt>{label}</dt>
                  <dd>{value}</dd>
                </div>
              ))}
            </dl>
          </LcarsCard>
        )}

        {quaternion && (
          <LcarsCard title="Quaternion" tone="violet">
            <p class="lcars-readout">{formatQuaternion(quaternion)}</p>
          </LcarsCard>
        )}

        {angularVelocityRows.length > 0 && (
          <LcarsCard title="Angular velocity" subtitle="rad/s" tone="cyan">
            <dl class="lcars-list lcars-list--columns">
              {angularVelocityRows.map(({ label, value }) => (
                <div class="lcars-list__item" key={`angVel-${label}`}>
                  <dt>{label}</dt>
                  <dd>{value}</dd>
                </div>
              ))}
            </dl>
          </LcarsCard>
        )}

        {linearAccelerationRows.length > 0 && (
          <LcarsCard title="Linear acceleration" subtitle="m/s²" tone="teal">
            <dl class="lcars-list lcars-list--columns">
              {linearAccelerationRows.map(({ label, value }) => (
                <div class="lcars-list__item" key={`linAcc-${label}`}>
                  <dt>{label}</dt>
                  <dd>{value}</dd>
                </div>
              ))}
            </dl>
          </LcarsCard>
        )}
      </div>
    </LcarsPanel>
  );
}

export default ImuReadout;
