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
