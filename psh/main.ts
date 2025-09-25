#!/usr/bin/env -S deno run -A

import { Command } from "@cliffy/command";

import { setup } from "./setup.ts";
import { printEnvSource } from "./psh_env.ts";
import { runInstallRos2, runInstallDocker } from "./install.ts";
import { systemdGenerate, systemdInstall, systemdUninstall } from "./systemd.ts";
import { uninstallPsh } from "./uninstall.ts";
import { clean } from "./clean.ts";
import { printSummaryTable } from "./cli_summary.ts";
import { runModuleScript } from "./mod.ts";


if (import.meta.main) {
  await new Command()
    .name("psh")
    .description("Psyched CLI")
    .version("v1.0.0")
    .globalOption("-d, --debug", "Enable debug output.")
    .action(async () => {
      await printSummaryTable();
    })
    .command("install", "Install psh to /usr/bin/psh")
    .action(async () => await uninstallPsh())
    .command("setup", "Setup host(s): configure modules and install dependencies (ros2, docker) as specified in host TOML. Usage: psh setup [host1] [host2] ... (default: current host)")
    .arguments("[host:string] [...hosts:string]")
    .action(async (options: Record<string, unknown>, host?: string, ...hosts: (string | undefined)[]) => {
      // Setup the specified hosts, or default to current host if none provided
      const targets = [host, ...hosts].filter(Boolean);
      if (targets.length === 0) {
        await setup(options, []);
      } else {
        for (const h of targets) {
          await setup(options, [h]);
        }
      }
    })
    .command("dep", "Install system dependencies (ros2, docker). Usage: psh dep <ros2|docker>")
    .alias("dependency")
    .alias("dependencies")
    .arguments("<dep:string>")
    .action(async (_options: Record<string, unknown>, dep: string) => {
      const depNorm = dep.toLowerCase();
      if (["ros2", "ros", "r"].includes(depNorm)) {
        await runInstallRos2();
        return;
      }
      if (["docker", "d"].includes(depNorm)) {
        await runInstallDocker();
        return;
      }
      console.error(`Unknown dependency: ${dep}`);
      Deno.exit(2);
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
    .command("env", "Print shell code to source ROS2 and workspace setup scripts for use with 'source $(psh env)'")
    .alias("environment")
    .action(async () => {
      await printEnvSource();
    })
    .command("mod", "Run module action (setup, launch, etc). Usage: psh mod <module> [action]")
    .alias("module")
    .arguments("<module:string> [action:string]")
    .action(async (_options: Record<string, unknown>, module: string, action?: string) => {
      await runModuleScript(module, action);
    })
    .command("clean", "Undo work made by psh and remove generated garbage")
    .arguments("[target:string]")
    .action(async (_options: Record<string, unknown>, ...args: (string | undefined)[]) => {
      const target = args && args[0] ? String(args[0]) : 'all';
      if (target === 'install') {
        await uninstallPsh();
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
