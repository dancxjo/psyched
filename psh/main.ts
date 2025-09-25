#!/usr/bin/env -S deno run -A

import { Command } from "@cliffy/command";
import { setup } from "./setup.ts";

import { repoPath, $ } from "./util.ts";
import { runInstallRos2, runInstallDocker } from "./install.ts";
import { systemdGenerate, systemdInstall } from "./systemd.ts";
import { uninstallPsh } from "./uninstall.ts";
import { clean } from "./clean.ts";
import { colors } from "@cliffy/ansi/colors";
import { printSummaryTable } from "./cli_summary.ts";
import { systemdUninstall } from "./systemd_uninstall.ts";




if (import.meta.main) {
  // Import ansi and table from cliffy
  const { Table } = await import("@cliffy/table");
  // Use statically imported colors from @cliffy/ansi

  await new Command()
    .name("psh")
    .description("Psyched CLI")
    .version("v1.0.0")
    .globalOption("-d, --debug", "Enable debug output.")
    .action(async () => {
      // Print a colored welcome banner
      console.log(colors.bold.underline.rgb24("Welcome to Psyched Robotics", 0x00bfff));
      console.log(colors.gray("─────────────────────────────────────────────"));

      await printSummaryTable();
    })
    .command("install", "Install psh to /usr/bin/psh")
    .action(async () => await uninstallPsh())
    .command("setup", "Setup sub-command.")
    .arguments("[target:string] [...rest:string]")
    .action(async (options: Record<string, unknown>, ...args: (string | undefined)[]) => {
      // Support multiple setup targets, e.g. `psh setup ros2 docker`.
      // If no known targets are provided, fall back to the existing setup() behaviour.
      if (!args || args.length === 0) {
        await setup(options, args as unknown[]);
        return;
      }

      const requested = args.filter(Boolean).map((a) => String(a));

      // Run matching installers in the order they were provided.
      let ranAny = false;
      for (const t of requested) {
        if (t === 'ros2') {
          await runInstallRos2();
          ranAny = true;
          continue;
        }
        if (t === 'docker') {
          await runInstallDocker();
          ranAny = true;
          continue;
        }
      }

      // If we didn't run any known installers, pass through to setup() for
      // backward compatibility.
      if (!ranAny) {
        await setup(options, args as unknown[]);
      }
    })
    .command("systemd", "Manage systemd units for this host")
    .alias("sys")
    .arguments("[action:string]")
    .action(async (_options: Record<string, unknown>, ...args: (string | undefined)[]) => {
      const action = args[0] || 'generate';
      if (action === '*' || action === 'generate' || action === 'gen') {
        await systemdGenerate();
        return;
      }
      if (action === 'install') {
        await systemdInstall();
        return;
      }
      console.error(`Unknown systemd action: ${action}`);
      Deno.exit(2);
    })
    // (removed unused 'bar' command)
    .command("env", "Print the path to the environment setup script (for sourcing)")
    .action(() => {
      // Print the absolute path to tools/setup_env.sh so callers can do:
      //   source $(psh env)
      console.log(repoPath('../tools/setup_env.sh'));
    })
    .command("clean", "Undo work made by psh and remove generated garbage")
    .arguments("[target:string]")
    .action(async (_options: Record<string, unknown>, ...args: (string | undefined)[]) => {
      const target = args && args[0] ? String(args[0]) : 'all';
      if (target === 'install') {
        await uninstallPsh(); // installPsh is now imported from uninstall.ts
        return;
      }
      if (target === 'systemd') {
        await systemdUninstall();
        return;
      }
      if (target === '*' || target === 'all') {
        await clean();
        await uninstallPsh();
        await systemdUninstall();
        return;
      }
      console.error(`Unknown clean target: ${target}`);
      Deno.exit(2);
    })
    .parse(Deno.args);
}
