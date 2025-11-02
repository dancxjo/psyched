import { ProvisionContext, ProvisionContextOptions } from "./deps/context.ts";
import { detectProvisionUser, parseGroupList } from "./user_groups.ts";

const DEFAULT_GROUP = "psyched-power";
const DEFAULT_RULE_PATH = "/etc/polkit-1/rules.d/60-psyched-host-power.rules";

export const HOST_POWER_ACTIONS = [
  "org.freedesktop.login1.power-off",
  "org.freedesktop.login1.power-off-multiple-sessions",
  "org.freedesktop.login1.power-off-ignore-inhibit",
  "org.freedesktop.login1.reboot",
  "org.freedesktop.login1.reboot-multiple-sessions",
  "org.freedesktop.login1.reboot-ignore-inhibit",
] as const;

function normalizeGroupName(name: string): string {
  return name.trim().toLowerCase();
}

export function buildHostPowerRule(
  group: string,
  actions: readonly string[] = HOST_POWER_ACTIONS,
): string {
  const allowed: string[] = [];
  const seen = new Set<string>();
  for (const action of actions) {
    const trimmed = action.trim();
    if (!trimmed || seen.has(trimmed)) continue;
    seen.add(trimmed);
    allowed.push(trimmed);
  }
  const safeGroup = group.trim();
  const lines = [
    "/* Psyched host power privileges */",
    "polkit.addRule(function(action, subject) {",
    "  const allowed = [",
    ...allowed.map((id) => `    "${id}",`),
    "  ];",
    `  if (allowed.indexOf(action.id) !== -1 && subject.isInGroup("${safeGroup}")) {`,
    "    return polkit.Result.YES;",
    "  }",
    "});",
    "",
  ];
  return lines.join("\n");
}

export interface EnsureHostPowerOptions extends ProvisionContextOptions {
  user?: string;
  group?: string;
  rulePath?: string;
}

export async function ensureHostPowerPrivileges(
  options: EnsureHostPowerOptions = {},
): Promise<void> {
  const context = new ProvisionContext(options);
  const targetGroup = (options.group ?? DEFAULT_GROUP).trim();
  const rulePath = (options.rulePath ?? DEFAULT_RULE_PATH).trim();
  await context.step(
    "Ensure host power management permissions",
    async (step) => {
      if (!targetGroup) {
        step.log("No host power group configured; skipping rule provisioning.");
        return;
      }

      await step.exec(["groupadd", "-f", targetGroup], {
        description: `ensure '${targetGroup}' group exists`,
        sudo: true,
      });

      const targetUser = options.user?.trim() || detectProvisionUser();
      if (!targetUser) {
        step.log(
          "Unable to detect provisioning user; skipping group membership update.",
        );
      } else if (targetUser === "root") {
        step.log(
          "Running as root; host power privileges are already available.",
        );
      } else {
        const { stdout } = await step.exec(["id", "-nG", targetUser], {
          description: `query existing groups for ${targetUser}`,
          stdoutOnSuccess: true,
        });
        const groups = parseGroupList(stdout);
        const normalizedTarget = normalizeGroupName(targetGroup);
        if (groups.includes(normalizedTarget)) {
          step.log(`${targetUser} already belongs to '${targetGroup}'.`);
        } else {
          await step.exec(["usermod", "-aG", targetGroup, targetUser], {
            description: `add ${targetUser} to '${targetGroup}'`,
            sudo: true,
          });
          step.log(
            `Added ${targetUser} to '${targetGroup}'. Log out/in or run 'newgrp ${targetGroup}' to refresh the session.`,
          );
        }
      }

      const desiredRule = buildHostPowerRule(targetGroup);
      let currentRule: string | null = null;
      try {
        currentRule = await Deno.readTextFile(rulePath);
      } catch (error) {
        if (
          error instanceof Deno.errors.NotFound ||
          error instanceof Deno.errors.PermissionDenied
        ) {
          currentRule = null;
        } else {
          throw error;
        }
      }

      if (currentRule === desiredRule) {
        step.log(`Polkit rule already present at ${rulePath}.`);
        return;
      }

      const script = [
        "set -euo pipefail",
        `cat <<'POLKIT' | install -o root -g root -m 0644 /dev/stdin '${rulePath}'`,
        desiredRule,
        "POLKIT",
      ].join("\n");
      await step.exec(["bash", "-lc", script], {
        description: `install polkit host power rule at ${rulePath}`,
        sudo: true,
      });
      step.log(`Installed host power polkit rule at ${rulePath}.`);
    },
  );
}

export const __test__ = {
  buildHostPowerRule,
};
