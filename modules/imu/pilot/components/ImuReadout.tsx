import { useMemo } from "preact/hooks";

import {
  Card,
  CONNECTION_STATUS_LABELS,
  Panel,
  toneFromConnection,
} from "@pilot/components/dashboard.tsx";
import {
  formatRelativeTime,
  formatTemperaturePair,
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
  const temperatureC = sensor.temperatureC;
  const hasTemperature =
    temperatureC !== undefined &&
    temperatureC !== null &&
    Number.isFinite(temperatureC);

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
    <Panel
      title={title}
      subtitle="Live inertial telemetry"
      accent="magenta"
      badges={[{
        label: connectionLabel,
        tone: toneFromConnection(connectionStatus),
      }]}
    >
      <div class="panel-grid">
        <Card title="Identity" tone="magenta">
          <dl class="stat-list">
            <div class="stat-list__item">
              <dt>Frame</dt>
              <dd>{sensor.frameId ?? "—"}</dd>
            </div>
            <div class="stat-list__item">
              <dt>Updated</dt>
              <dd>{formatRelativeTime(lastUpdate)}</dd>
            </div>
            {hasTemperature && (
              <div class="stat-list__item">
                <dt>Temperature</dt>
                <dd>{formatTemperaturePair(temperatureC)}</dd>
              </div>
            )}
          </dl>
          {sensor.status && <p class="note">{sensor.status}</p>}
        </Card>

        {eulerRows.length > 0 && (
          <Card title="Euler" subtitle="degrees" tone="violet">
            <dl class="stat-list stat-list--columns">
              {eulerRows.map(({ label, value }) => (
                <div class="stat-list__item" key={`euler-${label}`}>
                  <dt>{label}</dt>
                  <dd>{value}</dd>
                </div>
              ))}
            </dl>
          </Card>
        )}

        {quaternion && (
          <Card title="Quaternion" tone="violet">
            <p class="sensor-readout">{formatQuaternion(quaternion)}</p>
          </Card>
        )}

        {angularVelocityRows.length > 0 && (
          <Card title="Angular velocity" subtitle="rad/s" tone="cyan">
            <dl class="stat-list stat-list--columns">
              {angularVelocityRows.map(({ label, value }) => (
                <div class="stat-list__item" key={`angVel-${label}`}>
                  <dt>{label}</dt>
                  <dd>{value}</dd>
                </div>
              ))}
            </dl>
          </Card>
        )}

        {linearAccelerationRows.length > 0 && (
          <Card title="Linear acceleration" subtitle="m/s²" tone="teal">
            <dl class="stat-list stat-list--columns">
              {linearAccelerationRows.map(({ label, value }) => (
                <div class="stat-list__item" key={`linAcc-${label}`}>
                  <dt>{label}</dt>
                  <dd>{value}</dd>
                </div>
              ))}
            </dl>
          </Card>
        )}
      </div>
    </Panel>
  );
}

export default ImuReadout;
