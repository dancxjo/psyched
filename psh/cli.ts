import { Command } from "@cliffy/command";
import { printSummaryTable } from "./cli_summary.ts";
import { installPsh, runInstallDocker, runInstallRos2 } from "./install.ts";
import { printEnvSource } from "./psh_env.ts";
import { runModuleScript } from "./mod.ts";
import { cleanWorkspace } from "./clean.ts";
import {
  systemdGenerate,
  systemdInstall,
  systemdUninstall,
} from "./systemd.ts";
import { uninstallPsh } from "./uninstall.ts";
import { setupHosts } from "./setup.ts";

import { colconBuild, colconInstall } from "./colcon.ts";

export interface CliDeps {
  printSummaryTable(): Promise<void> | void;
  installPsh(): Promise<void> | void;
  uninstallPsh(): Promise<void> | void;
  setupHosts(hosts: string[]): Promise<void> | void;
  runInstallRos2(): Promise<void> | void;
  runInstallDocker(): Promise<void> | void;
  systemdGenerate(): Promise<void> | void;
  systemdInstall(): Promise<void> | void;
  systemdUninstall(): Promise<void> | void;
  printEnvSource(): Promise<void> | void;
  runModuleScript(module: string, action?: string): Promise<void> | void;
  cleanWorkspace(): Promise<void> | void;
  colconBuild(): Promise<void> | void;
  colconInstall(): Promise<void> | void;
}

const defaultDeps: CliDeps = {
  printSummaryTable,
  installPsh,
  uninstallPsh,
  setupHosts,
  runInstallRos2,
  runInstallDocker,
  systemdGenerate,
  systemdInstall,
  systemdUninstall,
  printEnvSource,
  runModuleScript,
  cleanWorkspace,
  colconBuild,
  colconInstall,
};

function normalize(text: string | undefined, fallback: string): string {
  return (text ?? fallback).toLowerCase();
}

export function createCli(overrides: Partial<CliDeps> = {}): Command {
  const deps: CliDeps = { ...defaultDeps, ...overrides } as CliDeps;

  const cli = new Command()
    .name("psh")
    .description("Psyched CLI")
    .version("v1.0.0")
    .throwErrors()
    .action(async () => {
      await deps.printSummaryTable();
    });

  cli.command("install")
    .description("Install the psh shim under /usr/bin/psh")
    .action(async () => {
      await deps.installPsh();
    });

  cli.command("build")
    .description("Run colcon build for the workspace")
    .action(async () => {
      await deps.colconBuild();
    });

  cli.command("colcon-install")
    .description("Run colcon install for the workspace")
    .action(async () => {
      await deps.colconInstall();
    });

  cli.command("setup")
    .description("Setup host modules and dependencies (default: current host)")
    .arguments("[host:string] [...hosts:string]")
    .action(
      async (_options, host?: string, ...rest: (string | undefined)[]) => {
        const targets = [host, ...rest].filter((value): value is string => Boolean(value));
        await deps.setupHosts(targets);
      },
    );

  cli.command("dep")
    .alias("dependency")
    .alias("dependencies")
    .description("Install a system dependency (ros2 or docker)")
    .arguments("<dep:string>")
    .action(async (_options, dep: string) => {
      const normalized = normalize(dep, "");
      if (["ros2", "ros", "r"].includes(normalized)) {
        await deps.runInstallRos2();
        return;
      }
      if (["docker", "d"].includes(normalized)) {
        await deps.runInstallDocker();
        return;
      }
      throw new Error(`Unknown dependency: ${dep}`);
    });

  cli.command("systemd")
    .description("Generate or install systemd unit files")
    .arguments("[action:string]")
    .alias("sys")
    .alias("service")
    .alias("srv")
    .action(async (_options, action?: string) => {
      const normalized = normalize(action, "generate");
      if (["*", "generate", "gen"].includes(normalized)) {
        await deps.systemdGenerate();
        return;
      }
      if (normalized === "install") {
        await deps.systemdInstall();
        return;
      }
      if (["uninstall", "remove", "rm"].includes(normalized)) {
        await deps.systemdUninstall();
        return;
      }
      throw new Error(`Unknown systemd action: ${action}`);
    });

  cli.command("env")
    .description(
      "Print shell code that sources ROS 2 and workspace setup scripts",
    )
    .action(async () => {
      await deps.printEnvSource();
    });

  cli.command("mod")
    .alias("module")
    .description("Run a module action (launch, shutdown, etc.)")
    .arguments("<module:string> [action:string]")
    .action(
      async (_options, module: string, action?: string) => {
        await deps.runModuleScript(module, action);
      },
    );

  cli.command("clean")
    .description("Remove generated artifacts or uninstall helpers")
    .arguments("[scope:string]")
    .action(async (_options, scope?: string) => {
      const target = normalize(scope, "workspace");
      if (["workspace", "ws", "src"].includes(target)) {
        await deps.cleanWorkspace();
        return;
      }
      if (["systemd", "units", "sys", "service", "srv"].includes(target)) {
        await deps.systemdUninstall();
        return;
      }
      if (["install", "psh", "shim"].includes(target)) {
        await deps.uninstallPsh();
        return;
      }
      if (["*", "all"].includes(target)) {
        await deps.cleanWorkspace();
        await deps.uninstallPsh();
        await deps.systemdUninstall();
        return;
      }
      throw new Error(`Unknown clean scope: ${scope}`);
    });

  return cli;
}
