import { colors } from "$cliffy/ansi/colors.ts";
import { workspaceRoot } from "./paths.ts";
import { runRosCommand } from "./ros_container.ts";

export interface ColconBuildInvocation {
  cwd: string;
  cmd: string[];
}

export type ColconRunner = (
  invocation: ColconBuildInvocation,
) => Promise<void> | void;

function splitPackagesAndArgs(args: string[]): {
  packages: string[];
  passthrough: string[];
} {
  const separatorIndex = args.indexOf("--");
  if (separatorIndex === -1) {
    return { packages: args, passthrough: [] };
  }
  const packages = args.slice(0, separatorIndex);
  const passthrough = args.slice(separatorIndex + 1);
  return { packages, passthrough };
}

export function createColconBuildInvocation(
  args: string[],
): ColconBuildInvocation {
  const { packages, passthrough } = splitPackagesAndArgs(args);
  const cmd = ["colcon", "build", "--symlink-install"];
  if (packages.length) {
    cmd.push("--packages-select", ...packages);
  }
  if (passthrough.length) {
    cmd.push("--", ...passthrough);
  }
  return {
    cwd: workspaceRoot(),
    cmd,
  };
}

function ensureWorkspaceExists(path: string): void {
  try {
    const stat = Deno.statSync(path);
    if (!stat.isDirectory) {
      throw new Error(
        `ROS workspace path '${path}' exists but is not a directory`,
      );
    }
  } catch (error) {
    if (error instanceof Deno.errors.NotFound) {
      throw new Error(
        `ROS workspace not found at '${path}'. Run 'psh clean' or 'psh mod setup' to initialise it.`,
      );
    }
    throw error;
  }
}

export async function buildWorkspace(
  args: string[],
  runner: ColconRunner = runColconBuild,
): Promise<void> {
  const invocation = createColconBuildInvocation(args);
  ensureWorkspaceExists(invocation.cwd);
  await runner(invocation);
}

async function runColconBuild(
  invocation: ColconBuildInvocation,
): Promise<void> {
  const [, ...colconArgs] = invocation.cmd;
  console.log(
    colors.cyan(
      `Running '${invocation.cmd.join(" ")}' in ${invocation.cwd}`,
    ),
  );
  await runRosCommand(["colcon", ...colconArgs], {
    cwd: invocation.cwd,
  });
}
