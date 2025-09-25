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
  systemdGenerate(units?: string[]): Promise<void> | void;
  systemdInstall(units?: string[]): Promise<void> | void;
  systemdUninstall(units?: string[]): Promise<void> | void;
  systemdDebug?(units?: string[]): Promise<void> | void;
  systemdEnable?(units?: string[]): Promise<void> | void;
  systemdDisable?(units?: string[]): Promise<void> | void;
  systemdStart?(units?: string[]): Promise<void> | void;
  systemdStop?(units?: string[]): Promise<void> | void;
  systemdReload?(units?: string[]): Promise<void> | void;
  systemdRestart?(units?: string[]): Promise<void> | void;
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
  systemdGenerate: () => systemdGenerate(),
  systemdInstall: (units?: string[]) => systemdInstall(units),
  systemdUninstall: () => systemdUninstall(),
  systemdDebug: () => { throw new Error("systemdDebug not implemented"); },
  systemdEnable: () => { throw new Error("systemdEnable not implemented"); },
  systemdDisable: () => { throw new Error("systemdDisable not implemented"); },
  systemdStart: () => { throw new Error("systemdStart not implemented"); },
  systemdStop: () => { throw new Error("systemdStop not implemented"); },
  systemdReload: () => { throw new Error("systemdReload not implemented"); },
  systemdRestart: () => { throw new Error("systemdRestart not implemented"); },
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
    .description("Psyched shell, a CLI for managing ROS2 workspaces and modules and for provisioning hosts in a robot")
    .version("v1.0.0")
    .throwErrors()
    .action(async () => {
      await deps.printSummaryTable();
    });

  cli.command("install")
    .alias("i")
    .description("Install the psh shim under /usr/bin/psh")
    .action(async () => {
      await deps.installPsh();
    });

  cli.command("provision")
    .alias("p")
    .description("Setup host modules and dependencies (default: current host)")
    .arguments("[host:string] [...hosts:string]")
    .action(
      async (_options, host?: string, ...rest: (string | undefined)[]) => {
        const targets = [host, ...rest].filter((value): value is string => Boolean(value));
        await deps.setupHosts(targets);
      },
    );

  cli.command("basics")
    .description("Install a system dependency (ros2 or docker). Called by `psh provision` automatically.")
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

  cli.command("module")
    .alias("mod")
    .alias("m")
    .description("Setup, launch, or shutdown modules")
    .arguments("[module:string] [action:string]")
    .action(
      async (_options, module: string = '*', action: string = 'list') => {
        await deps.runModuleScript(module, action);
      },
    );


  cli.command("build")
    .description("Run colcon build and install for the workspace")
    .action(async () => {
      // TODO: Clear out build/ and install/ folders first.
      try {
        await deps.colconBuild();
        // await deps.colconInstall();
      } catch (err: any) {
        // Provide a clearer hint when Deno lacks permissions to spawn processes
        const msg = String(err?.message || err);
        if (msg.includes("Requires --allow-run") || msg.includes("NotCapable") || msg.includes("allow-run")) {
          console.error("[psh] Deno permission error: running external commands requires Deno to be granted run permissions.");
          console.error("Try: deno run -A --config psh/deno.json psh/main.ts build");
          console.error("Or install the psh shim and run 'psh build' normally after installation.");
          Deno.exit(125);
        }
        throw err;
      }
    });



  // Systemd command: dispatch action and optional units as arguments
  const _sys = cli.command("systemd")
    .alias("sys")
    .alias("service")
    .alias("srv")
    .description("Manage systemd unit files (gen/install, debug, enable, disable, stop, start, reload, restart)")
    .arguments("[action:string] [...units:string]")
    .action(async (_options, action?: string, ...units: string[]) => {
      const act = normalize(action, "gen");
      if (["*", "gen", "generate"].includes(act)) {
        // generate now merges install (write + copy into systemd dir)
        await deps.systemdInstall(units.length ? units : undefined);
        return;
      }
      if (["uninstall", "remove", "rm"].includes(act)) {
        await deps.systemdUninstall(units.length ? units : undefined);
        return;
      }
      if (["debug", "dbg", "d"].includes(act)) {
        if (deps.systemdDebug) {
          await deps.systemdDebug(units.length ? units : undefined);
          return;
        }
        throw new Error("systemdDebug not implemented");
      }
      if (["enable", "en"].includes(act)) {
        if (deps.systemdEnable) {
          await deps.systemdEnable(units.length ? units : undefined);
          return;
        }
        throw new Error("systemdEnable not implemented");
      }
      if (["disable", "dis"].includes(act)) {
        if (deps.systemdDisable) {
          await deps.systemdDisable(units.length ? units : undefined);
          return;
        }
        throw new Error("systemdDisable not implemented");
      }
      if (["start"].includes(act)) {
        if (deps.systemdStart) {
          await deps.systemdStart(units.length ? units : undefined);
          return;
        }
        throw new Error("systemdStart not implemented");
      }
      if (["stop"].includes(act)) {
        if (deps.systemdStop) {
          await deps.systemdStop(units.length ? units : undefined);
          return;
        }
        throw new Error("systemdStop not implemented");
      }
      if (["reload", "reload-daemon", "rd"].includes(act)) {
        if (deps.systemdReload) {
          await deps.systemdReload(units.length ? units : undefined);
          return;
        }
        throw new Error("systemdReload not implemented");
      }
      if (["restart", "re"].includes(act)) {
        if (deps.systemdRestart) {
          await deps.systemdRestart(units.length ? units : undefined);
          return;
        }
        throw new Error("systemdRestart not implemented");
      }
      throw new Error(`Unknown systemd action: ${action}`);
    });

  cli.command("environment")
    .alias("env")
    .description(
      "Setup bash to use the ROS 2 and workspace environment",
    )
    .action(async () => {
      await deps.printEnvSource();
    });


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
