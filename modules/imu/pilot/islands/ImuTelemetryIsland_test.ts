import { assertAlmostEquals } from "$std/assert/assert_almost_equals.ts";
import { assertEquals } from "$std/assert/assert_equals.ts";
import { assertExists } from "$std/assert/assert_exists.ts";

import { __test__ } from "./ImuTelemetryIsland.tsx";

Deno.test("mapMessageToSample computes quaternion and Euler angles", () => {
  const result = __test__.mapMessageToSample({
    header: { stamp: { sec: 12, nanosec: 340_000_000 } },
    frame_id: "imu_frame",
    orientation: { x: 0, y: 0, z: 0.70710678, w: 0.70710678 },
    angular_velocity: { x: 0.1, y: -0.2, z: 0.3 },
    linear_acceleration: { x: 1, y: 2, z: 3 },
  });

  assertEquals(result.frameId, "imu_frame");
  assertExists(result.orientation);
  assertEquals(result.orientation?.quaternion, [0, 0, 0.70710678, 0.70710678]);
  assertExists(result.orientation?.euler);
  assertAlmostEquals(result.orientation!.euler!.roll ?? 0, 0, 1e-6);
  assertAlmostEquals(result.orientation!.euler!.pitch ?? 0, 0, 1e-6);
  assertAlmostEquals(result.orientation!.euler!.yaw ?? 0, 90, 1e-2);
  assertEquals(result.angularVelocity, { x: 0.1, y: -0.2, z: 0.3 });
  assertEquals(result.linearAcceleration, { x: 1, y: 2, z: 3 });
  assertEquals(result.lastUpdate, "1970-01-01T00:00:12.340Z");
});

Deno.test("mapMessageToSample omits undefined vectors", () => {
  const result = __test__.mapMessageToSample({
    header: { stamp: { sec: 1, nanosec: 0 } },
  });

  assertEquals(result.angularVelocity, undefined);
  assertEquals(result.linearAcceleration, undefined);
});
