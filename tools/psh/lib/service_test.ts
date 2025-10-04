import { assert, assertEquals } from "$std/testing/asserts.ts";
import { ServiceConfig, ServiceShellOptions, __test__, listServices } from "./service.ts";

const { buildShellArgs } = __test__;

type MinimalConfig = ServiceConfig & { compose: string };

Deno.test("buildShellArgs uses defaults from config", () => {
  const config: MinimalConfig = {
    compose: "docker-compose.yml",
    shell_command: ["/ros_entrypoint.sh", "bash"],
  };
  const args = buildShellArgs(
    "ros2",
    config,
    "/tmp/docker-compose.yml",
    "psyched-ros2",
  );
  assertEquals(args, [
    "docker",
    "compose",
    "-f",
    "/tmp/docker-compose.yml",
    "-p",
    "psyched-ros2",
    "exec",
    "-i",
    "-t",
    "ros2",
    "/ros_entrypoint.sh",
    "bash",
  ]);
});

Deno.test("buildShellArgs honors overrides", () => {
  const config: MinimalConfig = {
    compose: "docker-compose.yml",
    shell_service: "bridge",
    shell_user: "1000:1000",
    shell_command: ["bash"],
  };
  const options: ServiceShellOptions = {
    command: ["bash", "-lc", "echo hi"],
    service: "workspace",
    user: "2000:2000",
    interactive: false,
    tty: false,
  };
  const args = buildShellArgs(
    "ros2",
    config,
    "/etc/compose.yml",
    "ros2-dev",
    options,
  );
  assertEquals(args, [
    "docker",
    "compose",
    "-f",
    "/etc/compose.yml",
    "-p",
    "ros2-dev",
    "exec",
    "workspace",
    "bash",
    "-lc",
    "echo hi",
  ]);
});

Deno.test("listServices picks up ros2", () => {
  const services = listServices();
  assert(
    services.includes("ros2"),
    "ros2 service should be discoverable",
  );
});
