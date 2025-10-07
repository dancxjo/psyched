import { join } from "$std/path/mod.ts";

export class RebootRequiredError extends Error {
  constructor(message = "System reboot required before continuing.") {
    super(message);
    this.name = "RebootRequiredError";
  }
}

export interface EnsureRebootOptions {
  sentinelPath?: string;
  readBootTime?: () => number;
}

function defaultSentinelPath(): string {
  const override = Deno.env.get("PSYCHED_REBOOT_SENTINEL");
  if (override && override.length > 0) {
    return override;
  }
  const xdgState = Deno.env.get("XDG_STATE_HOME");
  if (xdgState && xdgState.length > 0) {
    return join(xdgState, "psyched", "reboot-required");
  }
  const home = Deno.env.get("HOME");
  if (home && home.length > 0) {
    return join(home, ".local", "state", "psyched", "reboot-required");
  }
  return join(Deno.cwd(), ".psyched", "reboot-required");
}

function detectBootTime(): number {
  try {
    const stat = Deno.readTextFileSync("/proc/stat");
    for (const line of stat.split("\n")) {
      if (line.startsWith("btime")) {
        const [, value] = line.trim().split(/\s+/);
        const parsed = Number(value);
        if (!Number.isNaN(parsed)) {
          return parsed;
        }
      }
    }
  } catch (_error) {
    // Fallback below.
  }
  return Math.floor(Date.now() / 1000);
}

function readSentinel(path: string): number | null {
  try {
    const content = Deno.readTextFileSync(path).trim();
    if (!content) {
      return null;
    }
    const parsed = Number(content);
    return Number.isNaN(parsed) ? null : parsed;
  } catch (error) {
    if (error instanceof Deno.errors.NotFound) {
      return null;
    }
    throw error;
  }
}

function removeSentinel(path: string): void {
  try {
    Deno.removeSync(path);
  } catch (error) {
    if (!(error instanceof Deno.errors.NotFound)) {
      throw error;
    }
  }
}

export function ensureRebootCompleted(
  options: EnsureRebootOptions = {},
): void {
  const sentinelPath = options.sentinelPath ?? defaultSentinelPath();
  const sentinelBootTime = readSentinel(sentinelPath);
  if (sentinelBootTime === null) {
    return;
  }
  const currentBootTime = options.readBootTime?.() ?? detectBootTime();
  if (currentBootTime <= sentinelBootTime) {
    throw new RebootRequiredError(
      "Reboot required before running module setup. Please reboot the system and rerun this command.",
    );
  }
  removeSentinel(sentinelPath);
}
