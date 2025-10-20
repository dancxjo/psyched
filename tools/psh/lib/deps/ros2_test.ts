import { assertEquals, assertThrows } from "$std/testing/asserts.ts";
import {
  buildColconProvisionPlan,
  buildRosBasePackages,
  determineRosDistro,
  renderRosContainerProfile,
  renderRosProfile,
  resolveRos2InstallPlan,
} from "./ros2.ts";
import { getRosDomainId } from "../ros_env.ts";
import { workspaceRoot } from "../paths.ts";

Deno.test("determineRosDistro respects env override", () => {
  const value = determineRosDistro({ env: { ROS_DISTRO: "jazzy" } });
  assertEquals(value, "jazzy");
});

Deno.test("determineRosDistro falls back to default", () => {
  const value = determineRosDistro({ env: {} });
  assertEquals(value, "kilted");
});

Deno.test("renderRosProfile emits consistent template", () => {
  const script = renderRosProfile("humble");
  const domainId = getRosDomainId();
  const expected =
    `# ROS 2 defaults provisioned by psh\nexport LANG=en_US.UTF-8\nexport RMW_IMPLEMENTATION=rmw_cyclonedds_cpp\nexport ROS_DOMAIN_ID=${domainId}\nexport ROS_LOCALHOST_ONLY=0\n# shellcheck disable=SC1091\n. /opt/ros/humble/setup.bash\n`;
  assertEquals(script, expected);
});

Deno.test("ROS base package set excludes colcon tooling", () => {
  const packages = buildRosBasePackages("jazzy");
  assertEquals(packages, [
    "ros-jazzy-ros-base",
    "ros-jazzy-rmw-cyclonedds-cpp",
    "python3-rosdep",
  ]);
});

Deno.test("colcon provisioning plan mirrors install script", () => {
  const plan = buildColconProvisionPlan("jazzy");
  assertEquals(plan.venvPath, "/opt/ros/jazzy/colcon-venv");
  assertEquals(plan.pythonBin, "/opt/ros/jazzy/colcon-venv/bin/python");
  assertEquals(plan.pipBin, "/opt/ros/jazzy/colcon-venv/bin/pip");
  assertEquals(plan.colconBin, "/opt/ros/jazzy/colcon-venv/bin/colcon");
  assertEquals(plan.symlinkPath, "/usr/local/bin/colcon");
  assertEquals(plan.packages, ["colcon-core", "colcon-common-extensions"]);
});

Deno.test("resolveRos2InstallPlan defaults to container mode", () => {
  const plan = resolveRos2InstallPlan();
  assertEquals(plan.mode, "container");
  assertEquals(plan.helperPath, "/usr/local/bin/ros2-container");
  assertEquals(plan.containerImage, "osrf/ros:humble-desktop");
  assertEquals(plan.containerName, "psyched-ros2");
  assertEquals(plan.networkMode, "host");
  assertEquals(plan.workspaceSource, workspaceRoot());
  assertEquals(plan.workspaceTarget, "/work");
  assertEquals(plan.passEnv, ["DISPLAY"]);
  assertEquals(plan.volumes, ["/tmp/.X11-unix:/tmp/.X11-unix:rw"]);
});

Deno.test("resolveRos2InstallPlan normalises container config", () => {
  const plan = resolveRos2InstallPlan({
    mode: "container",
    helper_path: "/opt/bin/ros2-container",
    container: {
      image: "osrf/ros:jazzy-desktop",
      name: "custom-ros2",
      network_mode: "bridge",
      workspace_source: "/data/psyched",
      workspace_target: "/workspace",
      user: "1001:1001",
      volumes: ["/var/run/docker.sock:/var/run/docker.sock"],
      pass_env: ["WAYLAND_DISPLAY"],
      env: { RMW_IMPLEMENTATION: "rmw_cyclonedds_cpp" },
    },
  });
  assertEquals(plan.mode, "container");
  assertEquals(plan.helperPath, "/opt/bin/ros2-container");
  assertEquals(plan.containerImage, "osrf/ros:jazzy-desktop");
  assertEquals(plan.containerName, "custom-ros2");
  assertEquals(plan.networkMode, "bridge");
  assertEquals(plan.workspaceSource, "/data/psyched");
  assertEquals(plan.workspaceTarget, "/workspace");
  assertEquals(plan.user, "1001:1001");
  assertEquals(plan.volumes, [
    "/tmp/.X11-unix:/tmp/.X11-unix:rw",
    "/var/run/docker.sock:/var/run/docker.sock",
  ]);
  assertEquals(plan.passEnv, ["DISPLAY", "WAYLAND_DISPLAY"]);
  assertEquals(plan.env, { RMW_IMPLEMENTATION: "rmw_cyclonedds_cpp" });
});

Deno.test("resolveRos2InstallPlan allows opting into native mode", () => {
  const plan = resolveRos2InstallPlan({ mode: "native" });
  assertEquals(plan.mode, "native");
  assertEquals(plan.helperPath, "/usr/local/bin/ros2-container");
  assertEquals(plan.volumes, []);
  assertEquals(plan.passEnv, []);
  assertEquals(plan.env, {});
});

Deno.test("resolveRos2InstallPlan rejects unsupported mode", () => {
  assertThrows(() => {
    resolveRos2InstallPlan({ mode: "invalid" });
  }, Error, "Unsupported ros2 installer mode");
});

Deno.test("renderRosContainerProfile emits helper alias", () => {
  const script = renderRosContainerProfile({
    helperPath: "/usr/local/bin/ros2-container",
    rosDistro: "humble",
  });
  const domainId = getRosDomainId();
  const expected = [
    "# ROS 2 container defaults provisioned by psh",
    "export LANG=en_US.UTF-8",
    "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp",
    `export ROS_DOMAIN_ID=${domainId}`,
    "export ROS_LOCALHOST_ONLY=0",
    "alias ros2-container='/usr/local/bin/ros2-container'",
    "",
  ].join("\n");
  assertEquals(script, expected);
});
