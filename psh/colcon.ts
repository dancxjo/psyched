import { $, type DaxTemplateTag, repoPath } from "./util.ts";

const withRosEnv = repoPath("../tools/with_ros_env.sh");

async function runColcon(
  runner: DaxTemplateTag,
  subcommand: string,
  args: string[],
  successMessage: string,
  errorLabel: string,
): Promise<void> {
  const absSrc = repoPath("../src");
  const builder = runner`${withRosEnv} ${[
    "colcon",
    subcommand,
    ...args,
    "--base-paths",
    absSrc,
  ]}`
    .stdout("inherit")
    .stderr("inherit");
  const result = await builder.noThrow();
  if (result.code !== 0) {
    console.error(`[psh] ${errorLabel} failed with code ${result.code ?? 1}`);
    Deno.exit(result.code ?? 1);
  }
  console.log(successMessage);
}

export async function colconBuild(runner: DaxTemplateTag = $): Promise<void> {
  await runColcon(
    runner,
    "build",
    ["--symlink-install"],
    "Colcon build complete.",
    "colcon build",
  );
}

export function colconInstall(): void {
  // Historically we attempted to run `colcon install` here, however the
  // `colcon` CLI does not provide an `install` subcommand. The workspace is
  // installed as part of `colcon build` (and using --symlink-install).
  // Keep this function for backwards compatibility but make it a no-op.
  console.log("[psh] Skipping 'colcon install' (not supported); ensure 'colcon build' was run instead.");
}
