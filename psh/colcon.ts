import { $, type DaxTemplateTag, repoPath } from "./util.ts";

async function runColcon(
  runner: DaxTemplateTag,
  subcommand: string,
  args: string[],
  successMessage: string,
  errorLabel: string,
): Promise<void> {
  const absSrc = repoPath("../src");
  const builder = runner`colcon ${[
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

export async function colconInstall(runner: DaxTemplateTag = $): Promise<void> {
  await runColcon(
    runner,
    "install",
    [],
    "Colcon install complete.",
    "colcon install",
  );
}
