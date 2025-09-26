// Shared utilities for psh
// Usage: import { repoPath, $ } from "./util.ts";

// @ts-ignore runtime import via jsr
import $ from "@david/dax";

/**
 * The Dax template tag type used throughout psh for shell commands.
 *
 * @example
 * ```ts
 * import { type DaxTemplateTag, $ } from "./util.ts";
 * const run: DaxTemplateTag = $;
 * await run`echo hello`;
 * ```
 */
export type DaxTemplateTag = typeof $;

/**
 * Command builder type returned by the Dax template tag.
 */
export type DaxCommandBuilder = ReturnType<typeof $>;

export function repoPath(relative: string): string {
  // Resolve a path relative to this file (psh/...). Example: '../tools/install_ros2.sh'
  return decodeURIComponent(new URL(relative, import.meta.url).pathname);
}

export { $ };

/**
 * Run a shell command while streaming output to the console and tee'ing
 * both stdout and stderr to temporary files so the caller can inspect them
 * after the command finishes. This preserves live streaming while ensuring
 * stderr is always available for error reports.
 *
 * Note: Uses bash process substitution and stdbuf to encourage line-buffered
 * output. Caller should be aware this requires a POSIX shell that supports
 * the used features (the environment already assumes bash).
 */
export async function runWithStreamingTee(
  command: string,
  _opts: { label?: string } = {},
): Promise<{ code: number; stdout: string; stderr: string }> {
  const shell = Deno.env.get("SHELL") || "/bin/bash";
  const outFile = await Deno.makeTempFile({ prefix: "psh-out-", suffix: ".log" });
  const errFile = await Deno.makeTempFile({ prefix: "psh-err-", suffix: ".log" });
  // Build a wrapper that runs the command with stdbuf to reduce buffering and
  // tees stdout/stderr to files while still streaming to the terminal.
  // Use set -o pipefail so the wrapper exit code reflects the command's exit.
  const wrapper = `set -o pipefail; stdbuf -oL -eL bash -c \"${command.replace(/\"/g, '\\"')}\" 2> >(tee \"${errFile}\" >&2) | tee \"${outFile}\"`;
  // Execute the wrapper via the user's shell so process substitution works.
  // Stream to the console live.
  const result = await $`${shell} -lc ${wrapper}`.stdout("inherit").stderr("inherit").noThrow();

  // Read captured files (best-effort) and then clean them up.
  let stdout = "";
  let stderr = "";
  try {
    stdout = await Deno.readTextFile(outFile);
  } catch { /* ignore */ }
  try {
    stderr = await Deno.readTextFile(errFile);
  } catch { /* ignore */ }
  try {
    await Deno.remove(outFile);
  } catch { /* ignore */ }
  try {
    await Deno.remove(errFile);
  } catch { /* ignore */ }
  return { code: result.code ?? 0, stdout, stderr };
}
