import { useEffect, useMemo, useReducer, useState } from "preact/hooks";

import { CONNECTION_STATUS_LABELS } from "@pilot/components/dashboard.tsx";
import { useCockpitTopic } from "@pilot/lib/cockpit.ts";
import ImuReadout, { type ImuSample } from "../components/ImuReadout.tsx";

type VectorLike = {
  x?: number | null;
  y?: number | null;
  z?: number | null;
};

type RosTimestamp =
  | { sec?: number; nanosec?: number }
  | { secs?: number; nanosecs?: number }
  | { sec?: number; nsec?: number }
  | { secs?: number; nsecs?: number };

type RosImuMessage = {
  header?: {
    frame_id?: string;
    frameId?: string;
    stamp?: RosTimestamp | string | number;
  };
  frame_id?: string;
  frameId?: string;
  orientation?: {
    x?: number | null;
    y?: number | null;
    z?: number | null;
    w?: number | null;
  };
  angular_velocity?: VectorLike;
  angularVelocity?: VectorLike;
  linear_acceleration?: VectorLike;
  linearAcceleration?: VectorLike;
  temperature?: number | null;
  status?: string;
  lastUpdate?: string | number | Date;
};

const DEFAULT_SAMPLE: ImuSample = {
  frameId: "imu_link",
  angularVelocity: { x: null, y: null, z: null },
  linearAcceleration: { x: null, y: null, z: null },
  status: "Awaiting telemetry",
};

export interface ImuTelemetryIslandProps {
  fallback?: ImuSample;
  title?: string;
  topic?: string;
}

/**
 * Interval in milliseconds used to keep the IMU readout refreshed even when no
 * new ROS messages are flowing. This drives the relative time badge so
 * operators can see telemetry staleness at a glance.
 */
const REFRESH_INTERVAL_MS = 1_000;

export default function ImuTelemetryIsland({
  fallback,
  title = "IMU Telemetry",
  topic = "/imu/data",
}: ImuTelemetryIslandProps) {
  const fallbackSample = useMemo<ImuSample>(() => ({
    ...DEFAULT_SAMPLE,
    ...(fallback ?? {}),
  }), [fallback]);

  const { data, status, error } = useCockpitTopic<RosImuMessage>(topic, {
    autoConnect: true, // Eagerly connect so the cockpit bridge begins streaming right away.
    replay: true,
  });

  const [sample, setSample] = useState<ImuSample>(fallbackSample);
  const [, forceRefresh] = useReducer((count: number) => count + 1, 0);

  useEffect(() => {
    setSample((previous) => ({ ...previous, ...fallbackSample }));
  }, [fallbackSample]);

  useEffect(() => {
    if (!data) return;
    const mapped = mapMessageToSample(data);
    if (Object.keys(mapped).length === 0) {
      return;
    }
    if (mapped.lastUpdate === undefined || mapped.lastUpdate === null) {
      mapped.lastUpdate = new Date();
    }
    setSample((previous) => ({ ...previous, ...mapped }));
  }, [data]);

  useEffect(() => {
    if (
      typeof globalThis.setInterval !== "function" ||
      typeof globalThis.clearInterval !== "function"
    ) {
      return;
    }
    const interval = globalThis.setInterval(() => {
      // Force a re-render so relative timestamps continue updating even when
      // telemetry is momentarily quiet.
      forceRefresh();
    }, REFRESH_INTERVAL_MS);
    return () => globalThis.clearInterval(interval);
  }, [forceRefresh]);

  const connectionLabel = CONNECTION_STATUS_LABELS[status] ?? "Unknown";
  const statusText = useMemo(() => (
    composeStatus(sample.status, connectionLabel, error)
  ), [sample.status, connectionLabel, error]);

  const sensor = useMemo<ImuSample>(() => ({
    ...sample,
    status: statusText,
  }), [sample, statusText]);

  return (
    <ImuReadout
      title={title}
      sensor={sensor}
      connectionStatus={status}
    />
  );
}

function mapMessageToSample(
  message: RosImuMessage | undefined,
): Partial<ImuSample> {
  if (!message) {
    return {};
  }

  const frameId = message.frameId ??
    message.frame_id ??
    message.header?.frameId ??
    message.header?.frame_id;

  const quaternion = toQuaternion(message.orientation);
  const euler = quaternion ? quaternionToEuler(quaternion) : undefined;
  const angularVelocity = vectorLikeToSample(
    message.angular_velocity ?? message.angularVelocity,
  );
  const linearAcceleration = vectorLikeToSample(
    message.linear_acceleration ?? message.linearAcceleration,
  );

  return {
    frameId,
    orientation: quaternion ? { quaternion, euler } : undefined,
    angularVelocity,
    linearAcceleration,
    temperatureC: message.temperature ?? undefined,
    lastUpdate: extractTimestamp(message) ?? message.lastUpdate ?? null,
    status: message.status ?? undefined,
  } satisfies Partial<ImuSample>;
}

function toQuaternion(
  orientation: RosImuMessage["orientation"] | undefined,
): [number, number, number, number] | undefined {
  if (!orientation) {
    return undefined;
  }

  const { x, y, z, w } = orientation;
  if (
    typeof x !== "number" ||
    typeof y !== "number" ||
    typeof z !== "number" ||
    typeof w !== "number"
  ) {
    return undefined;
  }

  return [x, y, z, w];
}

function quaternionToEuler(
  [x, y, z, w]: [number, number, number, number],
): { roll: number; pitch: number; yaw: number } {
  const sinrCosp = 2 * (w * x + y * z);
  const cosrCosp = 1 - 2 * (x * x + y * y);
  const roll = Math.atan2(sinrCosp, cosrCosp);

  const sinp = 2 * (w * y - z * x);
  const pitch = Math.abs(sinp) >= 1
    ? Math.sign(sinp) * Math.PI / 2
    : Math.asin(sinp);

  const sinyCosp = 2 * (w * z + x * y);
  const cosyCosp = 1 - 2 * (y * y + z * z);
  const yaw = Math.atan2(sinyCosp, cosyCosp);

  const toDegrees = (radians: number) => radians * (180 / Math.PI);
  return {
    roll: toDegrees(roll),
    pitch: toDegrees(pitch),
    yaw: toDegrees(yaw),
  };
}

function vectorLikeToSample(vector: VectorLike | undefined) {
  if (!vector) {
    return undefined;
  }

  const { x = null, y = null, z = null } = vector;
  if (x === null && y === null && z === null) {
    return undefined;
  }

  return { x, y, z };
}

function extractTimestamp(message: RosImuMessage): string | null {
  const stamp = message.header?.stamp;
  if (!stamp) {
    return null;
  }

  if (typeof stamp === "string") {
    return stamp;
  }

  if (typeof stamp === "number") {
    return new Date(stamp).toISOString();
  }

  if (typeof stamp !== "object") {
    return null;
  }

  const sec = (stamp as { sec?: number; secs?: number }).sec ??
    (stamp as { sec?: number; secs?: number }).secs;
  const nanosec = (stamp as { nanosec?: number; nanosecs?: number }).nanosec ??
    (stamp as { nsec?: number; nsecs?: number }).nsec ??
    (stamp as { nanosec?: number; nanosecs?: number }).nanosecs ??
    (stamp as { nsec?: number; nsecs?: number }).nsecs ?? 0;

  if (typeof sec !== "number") {
    return null;
  }

  const ms = Math.trunc(
    sec * 1000 + (typeof nanosec === "number" ? nanosec / 1_000_000 : 0),
  );
  return new Date(ms).toISOString();
}

function composeStatus(
  sampleStatus: string | undefined,
  connectionLabel: string,
  error: string | null,
): string {
  if (error) {
    return `Error: ${error}`;
  }
  const trimmed = sampleStatus?.trim();
  if (!trimmed) {
    return connectionLabel;
  }
  if (trimmed === connectionLabel) {
    return connectionLabel;
  }
  return `${trimmed} (${connectionLabel})`;
}

export const __test__ = {
  mapMessageToSample,
  quaternionToEuler,
  composeStatus,
};
