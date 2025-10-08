import { colors } from "$cliffy/ansi/colors.ts";
import { join } from "$std/path/mod.ts";
import { repoRoot } from "./paths.ts";

function shellQuote(value: string): string {
  return `'${value.replace(/'/g, `'"'"'`)}'`;
}

function pathExists(path: string): boolean {
  try {
    Deno.statSync(path);
    return true;
  } catch (error) {
    if (error instanceof Deno.errors.NotFound) return false;
    throw error;
  }
}

function workspaceCleanerPath(): string {
  return join(repoRoot(), "tools", "clean_workspace");
}

/**
 * Reset the ROS workspace using the repository's helper script.
 *
 * @example
 * ```ts
 * await resetWorkspace();
 * ```
 */
export async function resetWorkspace(): Promise<void> {
  const scriptPath = workspaceCleanerPath();
  if (!pathExists(scriptPath)) {
    console.log(
      colors.yellow(
        `Workspace cleaner not found at ${scriptPath}; skipping reset step.`,
      ),
    );
    return;
  }

  const script = `set -euo pipefail\n${shellQuote(scriptPath)}`;
  const process = new Deno.Command("bash", {
    args: ["-lc", script],
    cwd: repoRoot(),
    stdout: "inherit",
    stderr: "inherit",
  }).spawn();
  const status = await process.status;
  if (!status.success) {
    const code = status.code ?? "unknown";
    const signal = status.signal ?? "";
    const suffix = signal ? ` (signal ${signal})` : "";
    throw new Error(
      `Workspace cleanup failed with exit code ${code}${suffix}.`,
    );
  }
}

export const __test__ = {
  cleanerPath: workspaceCleanerPath,
};
