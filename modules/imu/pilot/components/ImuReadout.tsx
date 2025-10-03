import { Fragment } from "preact";
import { useMemo } from "preact/hooks";

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

const formatTimestamp = (value?: Date | string | null) => {
  if (!value) return "—";
  const date = value instanceof Date ? value : new Date(value);
  if (Number.isNaN(date.getTime())) {
    return "—";
  }
  return date.toLocaleString();
};

const sectionClass = "imu-readout__section";
const labelClass = "imu-readout__label";
const valueClass = "imu-readout__value";

export function ImuReadout({ title = "IMU Sensor", sensor }: ImuReadoutProps) {
  const euler = sensor.orientation?.euler;
  const quaternion = sensor.orientation?.quaternion;

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
  }, [sensor.angularVelocity?.x, sensor.angularVelocity?.y, sensor.angularVelocity?.z]);

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
  }, [sensor.linearAcceleration?.x, sensor.linearAcceleration?.y, sensor.linearAcceleration?.z]);

  return (
    <section class="imu-readout">
      <header class="imu-readout__header">
        <h2>{title}</h2>
        {sensor.status && <span class="imu-readout__status">{sensor.status}</span>}
      </header>

      <dl class={sectionClass}>
        <dt class={labelClass}>Frame</dt>
        <dd class={valueClass}>{sensor.frameId ?? "—"}</dd>

        <dt class={labelClass}>Last update</dt>
        <dd class={valueClass}>{formatTimestamp(sensor.lastUpdate)}</dd>

        {sensor.temperatureC !== undefined && sensor.temperatureC !== null && (
          <Fragment>
            <dt class={labelClass}>Temperature</dt>
            <dd class={valueClass}>{formatNumber(sensor.temperatureC, 1)} °C</dd>
          </Fragment>
        )}
      </dl>

      {eulerRows.length > 0 && (
        <dl class={sectionClass}>
          <dt class={`${labelClass} imu-readout__section-title`}>Euler (deg)</dt>
          {eulerRows.map(({ label, value }) => (
            <div class="imu-readout__row" key={`euler-${label}`}>
              <dt class={labelClass}>{label}</dt>
              <dd class={valueClass}>{value}</dd>
            </div>
          ))}
        </dl>
      )}

      {quaternion && (
        <dl class={sectionClass}>
          <dt class={`${labelClass} imu-readout__section-title`}>Quaternion</dt>
          <dd class={valueClass}>{formatQuaternion(quaternion)}</dd>
        </dl>
      )}

      {angularVelocityRows.length > 0 && (
        <dl class={sectionClass}>
          <dt class={`${labelClass} imu-readout__section-title`}>Angular velocity (rad/s)</dt>
          {angularVelocityRows.map(({ label, value }) => (
            <div class="imu-readout__row" key={`angVel-${label}`}>
              <dt class={labelClass}>{label}</dt>
              <dd class={valueClass}>{value}</dd>
            </div>
          ))}
        </dl>
      )}

      {linearAccelerationRows.length > 0 && (
        <dl class={sectionClass}>
          <dt class={`${labelClass} imu-readout__section-title`}>
            Linear acceleration (m/s²)
          </dt>
          {linearAccelerationRows.map(({ label, value }) => (
            <div class="imu-readout__row" key={`linAcc-${label}`}>
              <dt class={labelClass}>{label}</dt>
              <dd class={valueClass}>{value}</dd>
            </div>
          ))}
        </dl>
      )}
    </section>
  );
}

export default ImuReadout;
