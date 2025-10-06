import { assertEquals } from "$std/testing/asserts.ts";
import {
  buildColconProvisionPlan,
  buildRosBasePackages,
  determineRosDistro,
  renderRosProfile,
} from "./ros2.ts";

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
  const expected =
    `# ROS 2 defaults provisioned by psh\nexport LANG=en_US.UTF-8\nexport RMW_IMPLEMENTATION=rmw_cyclonedds_cpp\nexport ROS_DOMAIN_ID=0\nexport ROS_LOCALHOST_ONLY=0\n# shellcheck disable=SC1091\n. /opt/ros/humble/setup.bash\n`;
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
