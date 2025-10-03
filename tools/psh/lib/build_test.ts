import { assertEquals, assertRejects } from "$std/testing/asserts.ts";
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
});

Deno.test("colcon build invocation selects packages when provided", () => {
  const invocation = createColconBuildInvocation(["pilot", "imu"]);
  assertEquals(invocation.cmd, [
    "colcon",
    "build",
    "--symlink-install",
    "--packages-select",
    "pilot",
    "imu",
  ]);
});

Deno.test("colcon build invocation forwards arguments after separator", () => {
  const invocation = createColconBuildInvocation([
    "pilot",
    "--",
    "--cmake-args",
    "-DCMAKE_BUILD_TYPE=RelWithDebInfo",
  ]);
  assertEquals(invocation.cmd, [
    "colcon",
    "build",
    "--symlink-install",
    "--packages-select",
    "pilot",
    "--",
    "--cmake-args",
    "-DCMAKE_BUILD_TYPE=RelWithDebInfo",
  ]);
});

Deno.test("buildWorkspace delegates to provided runner", async () => {
  const tempWorkspace = Deno.makeTempDirSync();
  const originalWorkspace = Deno.env.get("PSYCHED_WORKSPACE_DIR");
  Deno.env.set("PSYCHED_WORKSPACE_DIR", tempWorkspace);
  try {
    const invocations: ColconBuildInvocation[] = [];
    await buildWorkspace(["pilot"], (invocation) => {
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
      "pilot",
    ]);
  } finally {
    if (originalWorkspace === undefined) {
      Deno.env.delete("PSYCHED_WORKSPACE_DIR");
    } else {
      Deno.env.set("PSYCHED_WORKSPACE_DIR", originalWorkspace);
    }
    Deno.removeSync(tempWorkspace, { recursive: true });
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
