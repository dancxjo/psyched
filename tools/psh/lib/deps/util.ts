/** Utility helpers shared by dependency installers. */

export interface ApplySudoOptions {
  /** When true, sudo will be prefixed unless already running as root. */
  requireSudo: boolean;
  /** Whether the current process already has root privileges. */
  isRoot: boolean;
  /** Absolute path to the sudo executable if available. */
  sudoPath?: string;
}

/**
 * Prefixes a command with sudo when required.
 *
 * @throws Error when sudo is required but unavailable.
 */
export function applySudo(
  args: string[],
  { requireSudo, isRoot, sudoPath }: ApplySudoOptions,
): string[] {
  if (!requireSudo || isRoot) {
    return [...args];
  }
  if (!sudoPath) {
    throw new Error(
      "sudo is required for this operation, but no sudo binary could be located.",
    );
  }
  return [sudoPath, ...args];
}
