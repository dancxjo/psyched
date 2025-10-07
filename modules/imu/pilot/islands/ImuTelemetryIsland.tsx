// @ts-nocheck -- Telemetry parsing depends on loosely-typed cockpit topic payloads.
import { useMemo } from "https://esm.sh/preact@10.22.0/hooks";
import {
  type ConnectionStatus,
  useCockpitTopic,
} from "../../../pilot/frontend/lib/cockpit.ts";
import ImuReadout, { type ImuSample } from "../components/ImuReadout.tsx";

const STATUS_LABELS: Record<ConnectionStatus, string> = {
  idle: "Idle",
  connecting: "Connecting",
  open: "Connected",
  closed: "Disconnected",
  error: "Error",
};

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

export default function ImuTelemetryIsland({
  fallback,
  title = "IMU Telemetry",
  topic = "/imu/data",
}: ImuTelemetryIslandProps) {
  const { data, status, error } = useCockpitTopic<RosImuMessage>(topic, {
    replay: true,
  });

  const sample = useMemo<ImuSample>(() => {
    const base = fallback ?? DEFAULT_SAMPLE;
    const mapped = mapMessageToSample(data);
    const connection = STATUS_LABELS[status] ?? "Unknown";
    const statusText = error ? `Error: ${error}` : connection;

    return {
      ...base,
      ...mapped,
      status: statusText,
    };
  }, [data, status, error, fallback]);

  return ImuReadout({
    title,
    sensor: sample,
  });
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
  const angularVelocity = vectorLikeToSample(
    message.angular_velocity ?? message.angularVelocity,
  );
  const linearAcceleration = vectorLikeToSample(
    message.linear_acceleration ?? message.linearAcceleration,
  );

  return {
    frameId,
    orientation: quaternion ? { quaternion } : undefined,
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
