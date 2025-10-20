import { CommandBuilder } from "$dax";
import { workspaceRoot } from "./paths.ts";

function pathExists(path: string): Deno.FileInfo | undefined {
  try {
    const info = Deno.statSync(path);
    return info;
  } catch (error) {
    if (error instanceof Deno.errors.NotFound) {
      return undefined;
    }
    throw error;
  }
}

function isExecutable(info: Deno.FileInfo): boolean {
  if (info.mode === undefined) {
    return true;
  }
  return (info.mode & 0o111) !== 0;
}

function candidateHelpers(): string[] {
  const envHelper = Deno.env.get("PSYCHED_ROS2_HELPER")?.trim();
  const candidates = [envHelper, "/usr/local/bin/ros2-container", "/opt/bin/ros2-container"];
  return candidates.filter((value): value is string => Boolean(value && value.length));
}

export function detectRosContainerHelper(): string | undefined {
  for (const candidate of candidateHelpers()) {
    const info = pathExists(candidate);
    if (!info) continue;
    if (info.isDirectory) continue;
    if (!isExecutable(info)) continue;
    return candidate;
  }
  return undefined;
}

export interface RosCommandPlan {
  command: string[];
  mode: "container" | "native";
  helper?: string;
}

export function buildRosCommand(args: string[]): RosCommandPlan {
  if (!args.length) {
    throw new Error("ROS command requires at least one argument");
  }
  const requestedMode = Deno.env.get("PSYCHED_ROS_MODE")?.toLowerCase();
  const helper = detectRosContainerHelper();

  if (requestedMode === "native") {
    return { command: [...args], mode: "native" };
  }

  if (requestedMode === "container" && !helper) {
    throw new Error(
      "PSYCHED_ROS_MODE=container but ros2-container helper not found. Rerun 'psh host setup' or install the ros2 container helper.",
    );
  }

  if (helper) {
    return { command: [helper, ...args], mode: "container", helper };
  }

  return { command: [...args], mode: "native" };
}

export interface RosCommandOptions {
  cwd?: string;
  env?: Record<string, string>;
}

export async function runRosCommand(
  args: string[],
  options: RosCommandOptions = {},
): Promise<void> {
  const plan = buildRosCommand(args);
  let builder = new CommandBuilder().command(plan.command)
    .stdin("inherit")
    .stdout("inherit")
    .stderr("inherit")
    .noThrow();
  builder = builder.cwd(options.cwd ?? workspaceRoot());
  if (options.env) {
    builder = builder.env(options.env);
  }
  const result = await builder.spawn();
  if (result.code !== 0) {
    const label = plan.mode === "container" && plan.helper
      ? `ros2 container helper '${plan.helper}'`
      : plan.command[0];
    throw new Error(`${label} exited with code ${result.code}`);
  }
}

export const __test__ = {
  candidateHelpers,
};
