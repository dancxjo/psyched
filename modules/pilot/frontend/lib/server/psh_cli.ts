import { join } from "$std/path/mod.ts";
import { pshRoot, repoRoot } from "./paths.ts";

const DENO_PATH = Deno.execPath();
const PSH_MAIN = join(pshRoot(), "main.ts");
const PSH_CONFIG = join(pshRoot(), "deno.json");

export async function runPsh(
  subcommand: string[],
  options: { env?: Record<string, string> } = {},
): Promise<void> {
  const command = new Deno.Command(DENO_PATH, {
    args: [
      "run",
      "-A",
      "--config",
      PSH_CONFIG,
      PSH_MAIN,
      ...subcommand,
    ],
    cwd: repoRoot(),
    env: options.env,
    stdin: "null",
    stdout: "inherit",
    stderr: "inherit",
  });
  const child = command.spawn();
  const status = await child.status;
  if (!status.success) {
    const description = [
      "psh",
      ...subcommand,
    ].join(" ");
    throw new Error(
      `${description} failed with code ${status.code ?? "unknown"}`,
    );
  }
}
