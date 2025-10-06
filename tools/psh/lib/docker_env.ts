import { CommandBuilder } from "$dax";

/**
 * Shell script executed inside the Ubuntu simulation container.
 *
 * It ensures the `pete` user exists before handing over an interactive login
 * shell for manual testing.
 */
const SIMULATION_SCRIPT = [
  "id -u pete >/dev/null 2>&1 || useradd -M -s /bin/bash -d /home/pete pete",
  "chown -R pete:pete /home/pete/psyched",
  "exec su - pete -s /bin/bash -l",
].join(" && ");

/**
 * Options for constructing the docker simulation command.
 */
export interface DockerSimulationOptions {
  /**
   * Workspace directory to mount into the container. Defaults to the current
   * working directory.
   */
  workspace?: string;
}

/**
 * Build the docker command used to spawn the Ubuntu 24.04 simulation
 * environment.
 *
 * @example
 * ```ts
 * const command = buildDockerSimulationCommand({ workspace: "/work" });
 * // command[0] === "docker"
 * ```
 */
export function buildDockerSimulationCommand(
  options: DockerSimulationOptions = {},
): string[] {
  const workspace = options.workspace ?? Deno.cwd();
  return [
    "docker",
    "run",
    "--rm",
    "-it",
    "-v",
    `${workspace}:/home/pete/psyched:rw`,
    "-w",
    "/home/pete/psyched",
    "--user",
    "root",
    "ubuntu:24.04",
    "bash",
    "-lc",
    SIMULATION_SCRIPT,
  ];
}

/**
 * Launch the Ubuntu simulation container, streaming stdio for interactive use.
 */
export async function launchDockerSimulation(
  options: DockerSimulationOptions = {},
): Promise<void> {
  const args = buildDockerSimulationCommand(options);
  const builder = new CommandBuilder().command(args)
    .stdin("inherit")
    .stdout("inherit")
    .stderr("inherit")
    .noThrow();
  const result = await builder.spawn();
  if (result.code !== 0) {
    throw new Error(`docker exited with code ${result.code}`);
  }
}

export const __test__ = {
  SIMULATION_SCRIPT,
};
