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
  systemdStart,
  systemdEnable,
  systemdDisable,
  systemdStop,
  systemdReload,
  systemdRestart,
  systemdDebug,
} from "./systemd.ts";
import { uninstallPsh } from "./uninstall.ts";
import { setupHosts, getHostModules } from "./setup.ts";

import { colconBuild, colconInstall } from "./colcon.ts";

import { downloadSpeechModels } from "./download_models.ts";
import {
  launchSpeechStack,
  stopSpeechStack,
  testSpeechStack,
  type SpeechStackLaunchOptions,
  type SpeechStackStopOptions,
  type SpeechStackTestOptions,
} from "./speech_stack.ts";
import { runSetupSpeech } from "./speech_setup.ts";

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
  runModuleScript(modules: string[] | string, action?: string): Promise<void> | void;
  cleanWorkspace(): Promise<void> | void;
  colconBuild(): Promise<void> | void;
  colconInstall(): Promise<void> | void;
  downloadSpeechModels(): Promise<void> | void;
  runSetupSpeech(): Promise<void> | void;
  launchSpeechStack(options?: SpeechStackLaunchOptions): Promise<void> | void;
  stopSpeechStack(options?: SpeechStackStopOptions): Promise<void> | void;
  testSpeechStack(options?: SpeechStackTestOptions): Promise<void> | void;
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
  systemdDebug: (units?: string[]) => systemdDebug(units),
  systemdEnable: (units?: string[]) => systemdEnable(units),
  systemdDisable: (units?: string[]) => systemdDisable(units),
  systemdStart: (units?: string[]) => systemdStart(units),
  systemdStop: (units?: string[]) => systemdStop(units),
  systemdReload: (units?: string[]) => systemdReload(units),
  systemdRestart: (units?: string[]) => systemdRestart(units),
  printEnvSource,
  runModuleScript,
  cleanWorkspace,
  colconBuild,
  colconInstall,
  downloadSpeechModels,
  runSetupSpeech,
  launchSpeechStack,
  stopSpeechStack,
  testSpeechStack,
};


function normalize(text: string | undefined, fallback: string): string {
  return (text ?? fallback).toLowerCase();
}

export function createCli(overrides: Partial<CliDeps> = {}): Command {

  const deps: CliDeps = { ...defaultDeps, ...overrides } as CliDeps;

  function isKnownAction(token: string | undefined): boolean {
    if (!token) return false;
    return allActionTokens.has(token.toLowerCase());
  }

  async function dispatchModuleOrService(
    defaultAction: string,
    actionArg: string | undefined,
    targetsArg: string[],
    opts: { preferSystemd: boolean },
  ): Promise<void> {
    let verb = actionArg;
    let targets = [...targetsArg];

    if (
      targets.length > 0 &&
      isKnownAction(targets[0]) &&
      (!verb || !isKnownAction(verb))
    ) {
      const [firstAction, ...rest] = targets;
      targets = verb ? [verb, ...rest] : rest;
      verb = firstAction;
    }

    const finalVerb = (verb ?? defaultAction);
    const normalizedVerb = finalVerb.toLowerCase();
    const canonicalAction = aliasToCanonical.get(normalizedVerb);

    if (canonicalAction) {
      const maybeUnits = targets.length ? targets : undefined;
      switch (canonicalAction) {
        case "generate":
          await deps.systemdGenerate(maybeUnits);
          return;
        case "install":
          await deps.systemdInstall(maybeUnits);
          return;
        case "uninstall":
          await deps.systemdUninstall(maybeUnits);
          return;
        case "debug":
          if (deps.systemdDebug) {
            await deps.systemdDebug(maybeUnits);
            return;
          }
          throw new Error("systemdDebug not implemented");
        case "enable":
          if (deps.systemdEnable) {
            await deps.systemdEnable(maybeUnits);
            return;
          }
          throw new Error("systemdEnable not implemented");
        case "disable":
          if (deps.systemdDisable) {
            await deps.systemdDisable(maybeUnits);
            return;
          }
          throw new Error("systemdDisable not implemented");
        case "start":
          if (deps.systemdStart) {
            await deps.systemdStart(maybeUnits);
            return;
          }
          throw new Error("systemdStart not implemented");
        case "stop":
          if (deps.systemdStop) {
            await deps.systemdStop(maybeUnits);
            return;
          }
          throw new Error("systemdStop not implemented");
        case "reload":
          if (deps.systemdReload) {
            await deps.systemdReload(maybeUnits);
            return;
          }
          throw new Error("systemdReload not implemented");
        case "restart":
          if (deps.systemdRestart) {
            await deps.systemdRestart(maybeUnits);
            return;
          }
          throw new Error("systemdRestart not implemented");
        default:
          throw new Error(`Unknown systemd action: ${canonicalAction}`);
      }
    }

    if (opts.preferSystemd) {
      const attempted = finalVerb;
      console.error(`[psh] Unknown systemd action: ${attempted}`);
      const swapped = targets
        .map((unit, index) => {
          const canonical = aliasToCanonical.get(unit.toLowerCase());
          return canonical
            ? { index, canonical, token: unit }
            : undefined;
        })
        .find((candidate): candidate is { index: number; canonical: SystemdAction; token: string } => Boolean(candidate));

      if (swapped) {
        const reorderedUnits = [
          attempted,
          ...targets.slice(0, swapped.index),
          ...targets.slice(swapped.index + 1),
        ].filter((value): value is string => Boolean(value));
        const suggestion = [
          "psh",
          "systemd",
          swapped.token,
          ...reorderedUnits,
        ].join(" ");
        console.error(`[psh] Did you mean '${suggestion}'?`);
      } else {
        const available = (Object.keys(systemdActionAliases) as SystemdAction[])
          .map((key) => preferredActionAlias[key])
          .join(", ");
        console.error(`[psh] Available actions: ${available}`);
      }
      Deno.exit(1);
      return;
    }

    let moduleList = targets;
    if (moduleList.length === 0) {
      const envHost = Deno.env.get("HOST");
      moduleList = envHost ? await getHostModules(envHost) : await getHostModules();
    }
    await deps.runModuleScript(moduleList.length ? moduleList : "*", finalVerb);
  }

  const cli = new Command()
    .name("psh")
    .description("Psyched shell, a CLI for managing ROS2 workspaces and modules and for provisioning hosts in a robot")
    .version("v1.0.0")
    .throwErrors()
    .action(async () => {
      await deps.printSummaryTable();
    });

  cli.command("help")
    .description("Show the psh command summary")
    .action(function () {
      const parent = this.getParent?.() ?? this;
      console.log(parent.getHelp());
    });

  const speech = new Command()
    .description("Manage or test the docker compose speech stack (ASR, TTS, LLM)");

  speech.command("launch")
    .description("Launch the docker compose speech stack")
    .alias("up")
    .option("-b, --build [build:boolean]", "Rebuild images before starting containers", { default: false })
    .action(async ({ build }: { build?: boolean }) => {
      await deps.launchSpeechStack({ build: Boolean(build) });
    });

  speech.command("down")
    .description("Stop the docker compose speech stack")
    .option("-v, --volumes [volumes:boolean]", "Remove named volumes when stopping", { default: false })
    .action(async ({ volumes }: { volumes?: boolean }) => {
      await deps.stopSpeechStack({ volumes: Boolean(volumes) });
    });

  speech.command("test")
    .description("Launch (if needed) and exercise the speech stack services")
    .option("-b, --build [build:boolean]", "Rebuild images before running the test suite", { default: false })
    .action(async ({ build }: { build?: boolean }) => {
      await deps.testSpeechStack({ build: Boolean(build) });
    });

  cli.command("speech", speech);
  // If `psh speech` is called with no args, run the launch action by default.
  speech.action(async () => {
    await deps.launchSpeechStack();
  });

  cli.command("models")
    .description("Download required models and set up folders for compose/speech-stack.compose.yml")
    .command("download")
    .description("Download all speech/LLM models needed for docker compose stack")
    .action(async () => {
      await deps.downloadSpeechModels();
    });

  cli.command("install")
    .alias("i")
    .description("Install the psh shim under /usr/bin/psh")
    .action(async () => {
      await deps.installPsh();
    });

  cli.command("provision")
    .alias("p")
    .alias("setup")
    .description("Setup host modules and dependencies (default: current host)")
    .arguments("[host:string] [...hosts:string]")
    .action(
      async (_options: unknown, host?: string, ...rest: (string | undefined)[]) => {
        const targets = [host, ...rest].filter((value): value is string => Boolean(value));
        await deps.setupHosts(targets);

        // Build the workspace between module setup and systemd installation.
        try {
          await deps.colconBuild();
        } catch (err) {
          console.error("[psh] Build step (psh build) failed:", String(err));
        }

        // Generate and enable systemd units (equivalent to `psh sys gen` and `psh sys enable`)
        try {
          // `systemdInstall` performs generation and copies units into systemd dir.
          await deps.systemdInstall();
          if (deps.systemdEnable) {
            await deps.systemdEnable();
          }
        } catch (err) {
          console.error("[psh] Systemd generation/enable failed:", String(err));
        }
      },
    );

  cli.command("basics")
    .alias("dep")
    .description("Install a system dependency (ros2 or docker). Called by `psh provision` automatically.")
    .arguments("<dep:string>")
    .action(async (_options: unknown, dep: string) => {
      const normalized = normalize(dep, "");
      if (["ros2", "ros", "r"].includes(normalized)) {
        await deps.runInstallRos2();
        return;
      }
      if (["docker", "d"].includes(normalized)) {
        await deps.runInstallDocker();
        return;
      }
      if (["speech", "speech-stack", "models"].includes(normalized)) {
        await deps.runSetupSpeech();
        return;
      }
      throw new Error(`Unknown dependency: ${dep}`);
    });

const knownModuleActions = new Set([
  "launch",
  "setup",
  "shutdown",
  "start",
  "stop",
  "list",
  "status",
]);

const systemdActionAliases = {
  generate: ["*", "gen", "generate"],
  install: ["install", "in"],
  uninstall: ["uninstall", "remove", "rm"],
  debug: ["debug", "dbg", "d", "status"],
  enable: ["enable", "en"],
  disable: ["disable", "dis"],
  start: ["start"],
  stop: ["stop"],
  reload: ["reload", "reload-daemon", "rd"],
  restart: ["restart", "re"],
} as const;

type SystemdAction = keyof typeof systemdActionAliases;

const preferredActionAlias: Record<SystemdAction, string> = {
  generate: "gen",
  install: "install",
  uninstall: "uninstall",
  debug: "debug",
  enable: "enable",
  disable: "disable",
  start: "start",
  stop: "stop",
  reload: "reload",
  restart: "restart",
};

const aliasToCanonical = new Map<string, SystemdAction>();
for (const [canonical, aliases] of Object.entries(systemdActionAliases)) {
  for (const alias of aliases) {
    aliasToCanonical.set(alias.toLowerCase(), canonical as SystemdAction);
  }
}

const allActionTokens = new Set<string>();
for (const action of knownModuleActions) {
  allActionTokens.add(action.toLowerCase());
}
for (const alias of aliasToCanonical.keys()) {
  allActionTokens.add(alias.toLowerCase());
}

  cli.command("module")
    .alias("mod")
    .alias("m")
    .description("Manage modules (setup, launch, shutdown) and related systemd services")
    .arguments("[action:string] [...targets:string]")
    .action(async (_options: unknown, action: string | undefined, ...targets: string[]) => {
      await dispatchModuleOrService("list", action, targets, { preferSystemd: false });
    });

  cli.command("systemd")
    .alias("sys")
    .alias("service")
    .alias("srv")
    .description("Alias of 'psh module' for service management; defaults to 'gen'")
    .arguments("[action:string] [...targets:string]")
    .action(async (_options: unknown, action: string | undefined, ...targets: string[]) => {
      await dispatchModuleOrService("gen", action, targets, { preferSystemd: true });
    });


  cli.command("build")
    .description("Run colcon build and install for the workspace")
    .action(async () => {
      // TODO: Clear out build/ and install/ folders first.
      try {
        await deps.colconBuild();
        // await deps.colconInstall();
      } catch (err: unknown) {
        // Provide a clearer hint when Deno lacks permissions to spawn processes
        const msg = typeof err === "object" && err !== null && "message" in err
          ? String((err as { message?: unknown }).message)
          : String(err);
        if (msg.includes("Requires --allow-run") || msg.includes("NotCapable") || msg.includes("allow-run")) {
          console.error("[psh] Deno permission error: running external commands requires Deno to be granted run permissions.");
          console.error("Try: deno run -A --config psh/deno.json psh/main.ts build");
          console.error("Or install the psh shim and run 'psh build' normally after installation.");
          Deno.exit(125);
        }
        throw err;
      }
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
    .action(async (_options: unknown, scope?: string) => {
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
