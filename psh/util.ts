// Shared utilities for psh
// Usage: import { repoPath, $ } from "./util.ts";

// @ts-ignore runtime import via jsr
import $ from "@david/dax";
import { dirname, fromFileUrl, join } from "@std/path";

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
  // Resolve a path relative to the repository root in the common case where
  // callers pass paths that start with ".." (e.g. "../tools/install_ros2.sh").
  // If callers intentionally pass a path starting with "./" we keep that
  // relative to the psh directory (this preserves psh-local references like
  // "./main.ts"). Absolute paths are returned unchanged.

  if (!relative) return "";

  // Return absolute paths unchanged
  if (relative.startsWith("/")) return relative;

  const utilDir = dirname(fromFileUrl(import.meta.url)); // .../psyched/psh

  // If the path starts with "./" resolve relative to the psh dir
  if (relative.startsWith("./")) {
    return join(utilDir, relative.slice(2));
  }

  // Handle one-or-more ".." path segments by walking up from utilDir
  if (relative.startsWith("..")) {
    let targetDir = utilDir;
    // Consume leading ../ segments and move targetDir up for each
    let rest = relative;
    while (rest.startsWith("../")) {
      targetDir = dirname(targetDir);
      rest = rest.slice(3);
    }
    // If nothing remains (e.g. ".." or "../"), return the targetDir
    if (!rest) return targetDir;
    return join(targetDir, rest);
  }

  // Fallback: treat as relative to the repo root (parent of psh)
  const repoRoot = dirname(utilDir);
  return join(repoRoot, relative);
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

/**
 * Coerce a TOML or CLI flag into a boolean value. Supports boolean,
 * string, and numeric inputs.
 */
export function asBoolean(value: unknown, fallback = false): boolean {
  if (typeof value === "boolean") return value;
  if (typeof value === "number") return value !== 0;
  if (typeof value === "string") {
    const normalized = value.trim().toLowerCase();
    if (["true", "1", "yes", "y", "on"].includes(normalized)) {
      return true;
    }
    if (["false", "0", "no", "n", "off"].includes(normalized)) {
      return false;
    }
  }
  return fallback;
}

/**
 * Read a boolean flag from a record while tolerating alternate key spellings.
 */
export function hasFlag(
  record: Record<string, unknown>,
  ...keys: string[]
): boolean {
  for (const key of keys) {
    if (Object.prototype.hasOwnProperty.call(record, key)) {
      return asBoolean(record[key]);
    }
  }
  return false;
}
