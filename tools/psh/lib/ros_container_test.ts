import { assertEquals, assertThrows } from "$std/testing/asserts.ts";
import { join } from "$std/path/mod.ts";
import { buildRosCommand, detectRosContainerHelper } from "./ros_container.ts";

async function createHelper(): Promise<string> {
  const dir = await Deno.makeTempDir({ prefix: "psh-ros-helper-" });
  const helperPath = join(dir, "ros2-container");
  await Deno.writeTextFile(helperPath, "#!/usr/bin/env bash\nexit 0\n");
  await Deno.chmod(helperPath, 0o755);
  return helperPath;
}

Deno.test("detectRosContainerHelper honours PSYCHED_ROS2_HELPER", async () => {
  const helper = await createHelper();
  const previous = Deno.env.get("PSYCHED_ROS2_HELPER");
  try {
    Deno.env.set("PSYCHED_ROS2_HELPER", helper);
    const detected = detectRosContainerHelper();
    assertEquals(detected, helper);
  } finally {
    if (previous === undefined) {
      Deno.env.delete("PSYCHED_ROS2_HELPER");
    } else {
      Deno.env.set("PSYCHED_ROS2_HELPER", previous);
    }
  }
});

Deno.test("buildRosCommand uses helper when available", async () => {
  const helper = await createHelper();
  const previousHelper = Deno.env.get("PSYCHED_ROS2_HELPER");
  const previousMode = Deno.env.get("PSYCHED_ROS_MODE");
  try {
    Deno.env.set("PSYCHED_ROS2_HELPER", helper);
    Deno.env.delete("PSYCHED_ROS_MODE");
    const plan = buildRosCommand(["colcon", "build"]);
    assertEquals(plan.mode, "container");
    assertEquals(plan.command[0], helper);
    assertEquals(plan.command.slice(1), ["colcon", "build"]);
  } finally {
    if (previousHelper === undefined) {
      Deno.env.delete("PSYCHED_ROS2_HELPER");
    } else {
      Deno.env.set("PSYCHED_ROS2_HELPER", previousHelper);
    }
    if (previousMode === undefined) {
      Deno.env.delete("PSYCHED_ROS_MODE");
    } else {
      Deno.env.set("PSYCHED_ROS_MODE", previousMode);
    }
  }
});

Deno.test("buildRosCommand honours native mode override", async () => {
  const helper = await createHelper();
  const previousHelper = Deno.env.get("PSYCHED_ROS2_HELPER");
  const previousMode = Deno.env.get("PSYCHED_ROS_MODE");
  try {
    Deno.env.set("PSYCHED_ROS2_HELPER", helper);
    Deno.env.set("PSYCHED_ROS_MODE", "native");
    const plan = buildRosCommand(["rosdep", "update"]);
    assertEquals(plan.mode, "native");
    assertEquals(plan.command, ["rosdep", "update"]);
  } finally {
    if (previousHelper === undefined) {
      Deno.env.delete("PSYCHED_ROS2_HELPER");
    } else {
      Deno.env.set("PSYCHED_ROS2_HELPER", previousHelper);
    }
    if (previousMode === undefined) {
      Deno.env.delete("PSYCHED_ROS_MODE");
    } else {
      Deno.env.set("PSYCHED_ROS_MODE", previousMode);
    }
  }
});

Deno.test("buildRosCommand errors when container requested but helper missing", () => {
  const previousMode = Deno.env.get("PSYCHED_ROS_MODE");
  const previousHelper = Deno.env.get("PSYCHED_ROS2_HELPER");
  try {
    Deno.env.set("PSYCHED_ROS_MODE", "container");
    Deno.env.delete("PSYCHED_ROS2_HELPER");
    assertThrows(() => buildRosCommand(["ros2", "topic", "list"]));
  } finally {
    if (previousMode === undefined) {
      Deno.env.delete("PSYCHED_ROS_MODE");
    } else {
      Deno.env.set("PSYCHED_ROS_MODE", previousMode);
    }
    if (previousHelper === undefined) {
      Deno.env.delete("PSYCHED_ROS2_HELPER");
    } else {
      Deno.env.set("PSYCHED_ROS2_HELPER", previousHelper);
    }
  }
});
