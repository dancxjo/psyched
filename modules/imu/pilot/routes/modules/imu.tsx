// @ts-nocheck
import ImuTelemetryIsland from "../../islands/ImuTelemetryIsland.tsx";
import type { ImuSample } from "../../components/ImuReadout.tsx";

const FALLBACK_SAMPLE: ImuSample = {
  frameId: "imu_link",
  orientation: {
    euler: { roll: 1.23, pitch: -0.42, yaw: 178.9 },
    quaternion: [0.01, 0.02, 0.03, 0.99],
  },
  angularVelocity: { x: 0.01, y: -0.02, z: 0.15 },
  linearAcceleration: { x: 0.05, y: 0.01, z: 9.81 },
  temperatureC: 42.5,
  status: "OK",
  lastUpdate: new Date().toISOString(),
};

export default function ImuModulePage() {
  return (
    <section class="content">
      <ImuTelemetryIsland fallback={FALLBACK_SAMPLE} />
    </section>
  );
}
