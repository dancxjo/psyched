import { ProvisionContext, ProvisionContextOptions } from "./deps/context.ts";

const ESSENTIAL_GROUPS = [
  "audio",
  "dialout",
  "i2c",
  "video",
  "gpio",
  "plugdev",
  "render",
] as const;

function normalizeGroupName(name: string): string {
  return name.trim().toLowerCase();
}

function uniqueNormalized(names: string[]): string[] {
  const seen = new Set<string>();
  for (const name of names) {
    const normalized = normalizeGroupName(name);
    if (normalized) {
      seen.add(normalized);
    }
  }
  return Array.from(seen);
}

/**
 * Resolve the canonical list of groups that grant hardware access to Pete.
 *
 * @example
 * ```ts
 * const groups = essentialGroups(["dialout", "i2c"]);
 * console.log(groups);
 * // => ["audio", "dialout", "i2c", "video", "gpio", "plugdev", "render"]
 * ```
 */
export function essentialGroups(extras: string[] = []): string[] {
  const combined = [
    ...ESSENTIAL_GROUPS,
    ...extras.map((name) => normalizeGroupName(name)),
  ];
  return uniqueNormalized(combined);
}

/**
 * Parse the `id -nG` output into a normalized list of group names.
 */
export function parseGroupList(raw: string): string[] {
  if (!raw) return [];
  return uniqueNormalized(raw.split(/\s+/));
}

/**
 * Compute which groups are absent from the user's membership list.
 *
 * @example
 * ```ts
 * const missing = computeMissingGroups(["audio", "video"], ["audio", "i2c"]);
 * console.log(missing);
 * // => ["i2c"]
 * ```
 */
export function computeMissingGroups(
  current: string[],
  required: string[],
): string[] {
  const currentSet = new Set(current.map(normalizeGroupName));
  const missing: string[] = [];
  for (const group of required) {
    const normalized = normalizeGroupName(group);
    if (!normalized || currentSet.has(normalized)) continue;
    currentSet.add(normalized);
    missing.push(normalized);
  }
  return missing;
}

export interface DetectUserOptions {
  env?: Record<string, string>;
}

/**
 * Identify the target user for provisioning. Prefers non-root accounts so we
 * avoid modifying the root user's groups unless explicitly requested.
 */
export function detectProvisionUser(
  options: DetectUserOptions = {},
): string | undefined {
  const env = options.env ?? Deno.env.toObject();
  const candidates = [
    env.SUDO_USER,
    env.USER,
    env.LOGNAME,
  ];
  for (const candidate of candidates) {
    const normalized = candidate?.trim();
    if (normalized && normalized !== "root") {
      return normalized;
    }
  }
  for (const candidate of candidates) {
    const normalized = candidate?.trim();
    if (normalized) return normalized;
  }
  try {
    const output = new Deno.Command("id", {
      args: ["-un"],
      stdout: "piped",
      stderr: "null",
    }).outputSync();
    if (output.success) {
      const user = new TextDecoder().decode(output.stdout).trim();
      if (user) return user;
    }
  } catch {
    // Ignore detection failures and fall through to undefined.
  }
  return undefined;
}

export interface EnsureUserGroupsOptions extends ProvisionContextOptions {
  user?: string;
  groups?: string[];
}

/**
 * Ensure the provisioning user belongs to the essential hardware access
 * groups so Pete's sensors and audio pipeline function without sudo.
 *
 * @example
 * ```ts
 * await ensureEssentialUserGroups();
 * await ensureEssentialUserGroups({ groups: ["i2c", "spi"] });
 * ```
 */
export async function ensureEssentialUserGroups(
  options: EnsureUserGroupsOptions = {},
): Promise<void> {
  const groups = essentialGroups(options.groups ?? []);
  if (!groups.length) return;

  const context = new ProvisionContext(options);
  const targetUser = options.user?.trim() || detectProvisionUser();

  await context.step("Ensure essential group memberships", async (step) => {
    if (!targetUser) {
      step.log("Unable to detect provisioning user; skipping group assignment.");
      return;
    }
    if (targetUser === "root") {
      step.log("Running as root; hardware groups are already accessible.");
      return;
    }

    const { stdout } = await step.exec(["id", "-nG", targetUser], {
      description: `query existing groups for ${targetUser}`,
      stdoutOnSuccess: true,
    });
    const currentGroups = parseGroupList(stdout);
    const requiredGroups = groups;
    const missingGroups = computeMissingGroups(currentGroups, requiredGroups);

    for (const group of requiredGroups) {
      await step.exec(["groupadd", "-f", group], {
        description: `ensure '${group}' group exists`,
        sudo: true,
      });
    }

    if (!missingGroups.length) {
      step.log(
        `${targetUser} already belongs to required groups (${requiredGroups.join(", ")}).`,
      );
      return;
    }

    const added: string[] = [];
    for (const group of missingGroups) {
      await step.exec(["usermod", "-aG", group, targetUser], {
        description: `add ${targetUser} to '${group}'`,
        sudo: true,
      });
      added.push(group);
      step.log(`Added ${targetUser} to '${group}'.`);
    }

    step.log(
      `Membership updated for ${targetUser}: ${added.join(", ")}. Log out/in or run 'newgrp' to refresh the session.`,
    );
  });
}

export const __test__ = {
  essentialGroups,
  parseGroupList,
  computeMissingGroups,
  detectProvisionUser,
};
