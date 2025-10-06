import { assertEquals } from "$std/assert/mod.ts";
import { buildDockerSimulationCommand } from "./docker_env.ts";

Deno.test("buildDockerSimulationCommand returns expected docker invocation", () => {
  const command = buildDockerSimulationCommand({ workspace: "/workdir" });
  const expected = [
    "docker",
    "run",
    "--rm",
    "-it",
    "-v",
    "/workdir:/home/pete/psyched:rw",
    "-w",
    "/home/pete/psyched",
    "--user",
    "root",
    "ubuntu:24.04",
    "bash",
    "-lc",
    "id -u pete >/dev/null 2>&1 || useradd -M -s /bin/bash -d /home/pete pete && chown -R pete:pete /home/pete/psyched && exec su - pete -s /bin/bash -l",
  ];
  assertEquals(command, expected);
});
