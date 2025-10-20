import { assert, assertEquals, assertRejects } from "$std/testing/asserts.ts";
import { join } from "$std/path/mod.ts";
import { workspaceRoot } from "./paths.ts";
import {
  buildWorkspace,
  ColconBuildInvocation,
  createColconBuildInvocation,
} from "./build.ts";

Deno.test("colcon build invocation defaults to workspace root", () => {
  const invocation = createColconBuildInvocation([]);
  assertEquals(invocation.cwd, workspaceRoot());
  assertEquals(invocation.cmd, ["colcon", "build", "--symlink-install"]);
  assertEquals(invocation.env, {});
});

Deno.test("colcon build invocation selects packages when provided", () => {
  const invocation = createColconBuildInvocation(["cockpit", "imu"]);
  assertEquals(invocation.cmd, [
    "colcon",
    "build",
    "--symlink-install",
    "--packages-select",
    "cockpit",
    "imu",
  ]);
  assertEquals(invocation.env, {});
});

Deno.test("colcon build invocation forwards arguments after separator", () => {
  const invocation = createColconBuildInvocation([
    "cockpit",
    "--",
    "--cmake-args",
    "-DCMAKE_BUILD_TYPE=RelWithDebInfo",
  ]);
  assertEquals(invocation.cmd, [
    "colcon",
    "build",
    "--symlink-install",
    "--packages-select",
    "cockpit",
    "--",
    "--cmake-args",
    "-DCMAKE_BUILD_TYPE=RelWithDebInfo",
  ]);
  assertEquals(invocation.env, {});
});

Deno.test("buildWorkspace delegates to provided runner", async () => {
  const tempWorkspace = Deno.makeTempDirSync();
  const tempRosDir = Deno.makeTempDirSync();
  const fakeRosSetup = join(tempRosDir, "setup.bash");
  const fakeAmentPath = join(tempRosDir, "fake-ament");
  // Provide a fake ROS environment so psyched::activate can succeed without
  // requiring the real distro inside the test container.
  Deno.writeTextFileSync(
    fakeRosSetup,
    [
      "#!/usr/bin/env bash",
      `export AMENT_PREFIX_PATH=\"${fakeAmentPath}\"`,
    ].join("\n"),
  );
  const originalWorkspace = Deno.env.get("PSYCHED_WORKSPACE_DIR");
  const originalRosSetup = Deno.env.get("PSYCHED_ROS_SETUP");
  const originalLogSilent = Deno.env.get("PSYCHED_LOG_SILENT");
  Deno.env.set("PSYCHED_WORKSPACE_DIR", tempWorkspace);
  Deno.env.set("PSYCHED_ROS_SETUP", fakeRosSetup);
  Deno.env.set("PSYCHED_LOG_SILENT", "1");
  try {
    const invocations: ColconBuildInvocation[] = [];
    await buildWorkspace(["cockpit"], (invocation) => {
      invocations.push(invocation);
    });
    assertEquals(invocations.length, 1);
    const invocation = invocations[0];
    assertEquals(invocation.cwd, tempWorkspace);
    assertEquals(invocation.cmd, [
      "colcon",
      "build",
      "--symlink-install",
      "--packages-select",
      "cockpit",
    ]);
    const rosDistro = invocation.env.ROS_DISTRO;
    assert(
      typeof rosDistro === "string" && rosDistro.length > 0,
      "expected ROS_DISTRO to be populated in invocation environment",
    );
    assertEquals(
      invocation.env.AMENT_PREFIX_PATH,
      fakeAmentPath,
      "expected ROS environment to expose AMENT_PREFIX_PATH",
    );
  } finally {
    if (originalWorkspace === undefined) {
      Deno.env.delete("PSYCHED_WORKSPACE_DIR");
    } else {
      Deno.env.set("PSYCHED_WORKSPACE_DIR", originalWorkspace);
    }
    if (originalRosSetup === undefined) {
      Deno.env.delete("PSYCHED_ROS_SETUP");
    } else {
      Deno.env.set("PSYCHED_ROS_SETUP", originalRosSetup);
    }
    if (originalLogSilent === undefined) {
      Deno.env.delete("PSYCHED_LOG_SILENT");
    } else {
      Deno.env.set("PSYCHED_LOG_SILENT", originalLogSilent);
    }
    Deno.removeSync(tempWorkspace, { recursive: true });
    Deno.removeSync(tempRosDir, { recursive: true });
  }
});

Deno.test("buildWorkspace raises a helpful error when workspace is missing", async () => {
  const tempRoot = Deno.makeTempDirSync();
  const missingWorkspace = join(tempRoot, "ros");
  const originalWorkspace = Deno.env.get("PSYCHED_WORKSPACE_DIR");
  Deno.env.set("PSYCHED_WORKSPACE_DIR", missingWorkspace);
  try {
    await assertRejects(
      () => buildWorkspace([], () => Promise.resolve()),
      Error,
      "ROS workspace",
      "expected informative error about workspace availability",
    );
  } finally {
    if (originalWorkspace === undefined) {
      Deno.env.delete("PSYCHED_WORKSPACE_DIR");
    } else {
      Deno.env.set("PSYCHED_WORKSPACE_DIR", originalWorkspace);
    }
    Deno.removeSync(tempRoot, { recursive: true });
  }
});
