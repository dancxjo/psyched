import { Command } from "$cliffy/command/mod.ts";
import { colors } from "$cliffy/ansi/colors.ts";
import { runWizard } from "./lib/wizard.ts";
import { HostConfigNotFoundError, provisionHost } from "./lib/host.ts";
import { buildWorkspace } from "./lib/build.ts";
import { cleanEnvironment } from "./lib/clean.ts";
import { launchDockerSimulation } from "./lib/docker_env.ts";
import {
  bringModulesDown,
  bringModulesUp,
  listModules,
  moduleStatuses,
  setupModules,
  teardownModules,
} from "./lib/module.ts";
import {
  bringServiceDown,
  bringServiceUp,
  listServices,
  openServiceShell,
  serviceStatuses,
  setupServices,
  teardownServices,
} from "./lib/service.ts";
import {
  defaultModuleTargets,
  defaultServiceTargets,
  resolveModuleTargets,
  resolveServiceTargets,
} from "./lib/host_targets.ts";
import { resolveTargetBatches } from "./lib/target_resolver.ts";
import {
  disableServiceSystemd,
  disableSystemd,
  enableServiceSystemd,
  enableSystemd,
  setupServiceSystemd,
  setupSystemd,
  startServiceSystemd,
  startSystemd,
  stopServiceSystemd,
  stopSystemd,
  teardownServiceSystemd,
  teardownSystemd,
} from "./lib/systemd.ts";
import { runSetupWorkflow, runTeardownWorkflow } from "./lib/workflow.ts";
import { publishString } from "./lib/ros.ts";

const version = "0.2.0";

async function main() {
  const root = new Command()
    .name("psh")
    .version(version)
    .description("psyched shell – Deno edition")
    .action(async () => {
      await runWizard();
    });

  root
    .command("say <text...:string>")
    .description("Publish a string to the /voice topic")
    .action(async (_, ...text: string[]) => {
      await publishString("/voice", text.join(" "));
    });

  root
    .command("setup")
    .description("Provision host, modules, and services")
    .option("--host <hostname:string>", "Host profile to apply")
    .option("-v, --verbose", "Show detailed provisioning logs")
    .option("--host-only", "Only run host provisioning")
    .option("--skip-modules", "Skip module setup stage")
    .option("--skip-services", "Skip service setup stage")
    .action(
      async (
        options: {
          host?: string;
          verbose?: boolean;
          hostOnly?: boolean;
          skipModules?: boolean;
          skipServices?: boolean;
        },
      ) => {
        const skipModules = Boolean(options.hostOnly || options.skipModules);
        const skipServices = Boolean(options.hostOnly || options.skipServices);
        await runSetupWorkflow({
          host: options.host,
          verbose: options.verbose,
          skipModules,
          skipServices,
        });
      },
    );

  root
    .command("teardown")
    .description("Tear down modules/services and reset the workspace")
    .option("--skip-modules", "Skip module teardown stage")
    .option("--skip-services", "Skip service teardown stage")
    .option("--skip-clean", "Skip workspace cleanup")
    .action(
      async (
        options: {
          skipModules?: boolean;
          skipServices?: boolean;
          skipClean?: boolean;
        },
      ) => {
        await runTeardownWorkflow({
          skipModules: Boolean(options.skipModules),
          skipServices: Boolean(options.skipServices),
          skipClean: Boolean(options.skipClean),
        });
      },
    );

  const hostCommand = new Command()
    .description("Host provisioning commands");

  root
    .command("docker")
    .description(
      "Launch an Ubuntu 24.04 container with the workspace mounted",
    )
    .option(
      "--workspace <path:string>",
      "Override the workspace directory mounted at /home/pete/psyched",
    )
    .action(async ({ workspace }: { workspace?: string }) => {
      await launchDockerSimulation({ workspace });
    });

  root
    .command("up [targets...:string]")
    .description("Launch modules and services")
    .option("--service", "Prefer services when resolving ambiguous names")
    .option("-v, --verbose", "Show detailed launch diagnostics")
    .action(
      async (
        { service, verbose }: { service?: boolean; verbose?: boolean },
        ...targets: string[]
      ) => {
        const { modules, services } = resolveTargetBatches(targets, {
          preferService: Boolean(service),
          defaults: {
            modules: defaultModuleTargets("launch"),
            services: defaultServiceTargets("up"),
          },
        });
        for (const svc of services) {
          await bringServiceUp(svc);
        }
        if (modules.length) {
          await bringModulesUp(modules, { verbose });
        }
      },
    );

  root
    .command("down [targets...:string]")
    .description("Stop modules and services")
    .option("--service", "Prefer services when resolving ambiguous names")
    .action(
      async (
        { service }: { service?: boolean },
        ...targets: string[]
      ) => {
        const { modules, services } = resolveTargetBatches(targets, {
          preferService: Boolean(service),
          defaults: {
            modules: defaultModuleTargets("launch"),
            services: defaultServiceTargets("up"),
          },
        });
        if (modules.length) {
          await bringModulesDown(modules);
        }
        for (const svc of services) {
          await bringServiceDown(svc);
        }
      },
    );

  root
    .command("build")
    .description("Run colcon build within the workspace")
    .arguments("[targets...:string]")
    .action(async (_, ...targets: string[]) => {
      await buildWorkspace(targets);
    });

  root
    .command("clean")
    .description("Tear down modules/services and reset the workspace")
    .option("--skip-modules", "Skip module teardown stage")
    .option("--skip-services", "Skip service teardown stage")
    .option("--skip-workspace", "Skip workspace reset stage")
    .action(
      async (
        options: {
          skipModules?: boolean;
          skipServices?: boolean;
          skipWorkspace?: boolean;
        },
      ) => {
        await cleanEnvironment({
          skipModules: Boolean(options.skipModules),
          skipServices: Boolean(options.skipServices),
          skipWorkspace: Boolean(options.skipWorkspace),
        });
      },
    );

  hostCommand
    .command("setup [hosts...:string]")
    .description("Provision the local or specified host(s)")
    .option("-v, --verbose", "Show detailed provisioning logs")
    .option("--include-modules", "Include module provisioning tasks")
    .option("--include-services", "Include service provisioning tasks")
    .action(
      async (
        {
          verbose,
          includeModules,
          includeServices,
        }: {
          verbose?: boolean;
          includeModules?: boolean;
          includeServices?: boolean;
        },
        ...hosts: string[]
      ) => {
        const options = { verbose, includeModules, includeServices };
        if (!hosts.length) {
          await provisionHost(undefined, options);
        } else {
          for (const host of hosts) {
            await provisionHost(host, options);
          }
        }
      },
    );

  root.command("host", hostCommand);

  const moduleCommand = new Command()
    .description("Module lifecycle commands");

  moduleCommand
    .command("list")
    .description("List available modules and status")
    .action(() => {
      console.log(colors.bold("Modules:"));
      for (const status of moduleStatuses()) {
        const state = status.status === "running"
          ? colors.green(`running${status.pid ? ` (pid ${status.pid})` : ""}`)
          : colors.yellow("stopped");
        console.log(`- ${status.name}: ${state}`);
      }
    });

  moduleCommand
    .command("setup [modules...:string]")
    .description("Run setup lifecycle for modules")
    .action(async (_, ...modules: string[]) => {
      const targets = resolveModuleTargets("setup", modules);
      await setupModules(targets);
    });

  moduleCommand
    .command("teardown [modules...:string]")
    .description("Run teardown lifecycle for modules")
    .action(async (_, ...modules: string[]) => {
      const targets = resolveModuleTargets("teardown", modules);
      await teardownModules(targets);
    });

  moduleCommand
    .command("up [modules...:string]")
    .description("Launch module processes")
    .option("-v, --verbose", "Show detailed launch diagnostics")
    .action(
      async ({ verbose }: { verbose?: boolean }, ...modules: string[]) => {
        const targets = resolveModuleTargets("launch", modules);
        await bringModulesUp(targets, { verbose });
      },
    );

  moduleCommand
    .command("down [modules...:string]")
    .description("Stop module processes")
    .action(async (_, ...modules: string[]) => {
      const targets = resolveModuleTargets("launch", modules);
      await bringModulesDown(targets);
    });

  root.command("mod", moduleCommand).alias("module");

  const serviceCommand = new Command()
    .description("Service lifecycle commands");

  serviceCommand
    .command("list")
    .description("List services and status")
    .action(async () => {
      console.log(colors.bold("Services:"));
      for (const status of await serviceStatuses()) {
        const state = status.status === "running"
          ? colors.green("running")
          : status.status === "stopped"
            ? colors.yellow("stopped")
            : colors.red("error");
        console.log(
          `- ${status.name}: ${state}${status.description ? ` – ${status.description}` : ""
          }`,
        );
      }
    });

  serviceCommand
    .command("setup [services...:string]")
    .description("Run setup for services")
    .action(async (_, ...services: string[]) => {
      const targets = resolveServiceTargets("setup", services);
      await setupServices(targets);
    });

  serviceCommand
    .command("teardown [services...:string]")
    .description("Run teardown for services")
    .action(async (_, ...services: string[]) => {
      const targets = resolveServiceTargets("teardown", services);
      await teardownServices(targets);
    });

  serviceCommand
    .command("up [services...:string]")
    .description("Start services")
    .action(async (_, ...services: string[]) => {
      const targets = resolveServiceTargets("up", services);
      for (const svc of targets) await bringServiceUp(svc);
    });

  serviceCommand
    .command("down [services...:string]")
    .description("Stop services")
    .action(async (_, ...services: string[]) => {
      const targets = resolveServiceTargets("up", services);
      for (const svc of targets) await bringServiceDown(svc);
    });

  serviceCommand
    .command("shell <service:string> [cmd...:string]")
    .description("Open an interactive shell inside a service container")
    .option(
      "-c, --container <name:string>",
      "Override the compose service to exec into",
    )
    .option(
      "-u, --user <user:string>",
      "Run the shell as a specific user (e.g. 1000:1000)",
    )
    .option("--no-tty", "Disable TTY allocation")
    .option("--no-stdin", "Do not attach stdin")
    .action(
      async (
        options: {
          container?: string;
          user?: string;
          noTty?: boolean;
          noStdin?: boolean;
        },
        service: string,
        ...cmd: string[]
      ) => {
        await bringServiceUp(service);
        const tty = options.noTty ? false : true;
        const interactive = options.noStdin ? false : true;
        await openServiceShell(service, {
          service: options.container,
          user: options.user,
          command: cmd.length ? cmd : undefined,
          tty,
          interactive,
        });
      },
    );

  root.command("srv", serviceCommand).alias("service");

  const systemCommand = new Command()
    .description("Systemd integration for modules and services");

  const addTargetOptions = (cmd: Command) =>
    cmd
      .option("--service", "Operate on a service instead of a module")
      .option("--module", "Force module mode (default)");

  systemCommand
    .command("setup <target:string>")
    .description("Generate a systemd unit for a module or service")
    .option("--service", "Operate on a service instead of a module")
    .option("--module", "Force module mode (default)")
    .action(async (options: SystemdTargetOptions, target: string) => {
      const kind = resolveSystemdTarget(target, options);
      if (kind === "module") {
        await setupSystemd(target);
      } else {
        await setupServiceSystemd(target);
      }
    });

  systemCommand
    .command("teardown <target:string>")
    .description("Remove the systemd unit for a module or service")
    .option("--service", "Operate on a service instead of a module")
    .option("--module", "Force module mode (default)")
    .action(async (options: SystemdTargetOptions, target: string) => {
      const kind = resolveSystemdTarget(target, options);
      if (kind === "module") {
        await teardownSystemd(target);
      } else {
        await teardownServiceSystemd(target);
      }
    });

  systemCommand
    .command("enable <target:string>")
    .description("Enable the systemd unit for a module or service")
    .option("--service", "Operate on a service instead of a module")
    .option("--module", "Force module mode (default)")
    .action(async (options: SystemdTargetOptions, target: string) => {
      const kind = resolveSystemdTarget(target, options);
      if (kind === "module") {
        await enableSystemd(target);
      } else {
        await enableServiceSystemd(target);
      }
    });

  systemCommand
    .command("disable <target:string>")
    .description("Disable the systemd unit for a module or service")
    .option("--service", "Operate on a service instead of a module")
    .option("--module", "Force module mode (default)")
    .action(async (options: SystemdTargetOptions, target: string) => {
      const kind = resolveSystemdTarget(target, options);
      if (kind === "module") {
        await disableSystemd(target);
      } else {
        await disableServiceSystemd(target);
      }
    });

  systemCommand
    .command("up <target:string>")
    .description("Start a module or service via systemd")
    .option("--service", "Operate on a service instead of a module")
    .option("--module", "Force module mode (default)")
    .action(async (options: SystemdTargetOptions, target: string) => {
      const kind = resolveSystemdTarget(target, options);
      if (kind === "module") {
        await startSystemd(target);
      } else {
        await startServiceSystemd(target);
      }
    });

  systemCommand
    .command("down <target:string>")
    .description("Stop a module or service via systemd")
    .option("--service", "Operate on a service instead of a module")
    .option("--module", "Force module mode (default)")
    .action(async (options: SystemdTargetOptions, target: string) => {
      const kind = resolveSystemdTarget(target, options);
      if (kind === "module") {
        await stopSystemd(target);
      } else {
        await stopServiceSystemd(target);
      }
    });

  root.command("sys", systemCommand);

  await root.parse(Deno.args);
}

interface SystemdTargetOptions {
  service?: boolean;
  module?: boolean;
}

type SystemdTargetKind = "module" | "service";

function resolveSystemdTarget(
  target: string,
  options: SystemdTargetOptions = {},
): SystemdTargetKind {
  if (options.service && options.module) {
    throw new Error("Cannot specify both --service and --module flags.");
  }
  const modules = listModules();
  const services = listServices();
  const isModule = modules.includes(target);
  const isService = services.includes(target);

  if (options.module) {
    if (!isModule) {
      throw new Error(`Unknown module '${target}'.`);
    }
    return "module";
  }

  if (options.service) {
    if (!isService) {
      throw new Error(`Unknown service '${target}'.`);
    }
    return "service";
  }

  if (isModule && !isService) return "module";
  if (!isModule && isService) return "service";
  if (isModule && isService) {
    throw new Error(
      `Target '${target}' matches both a module and a service. Use --module or --service to disambiguate.`,
    );
  }

  throw new Error(`Unknown module or service '${target}'.`);
}

async function handleTopLevelError(error: unknown): Promise<void> {
  const reason = error instanceof Error ? error : new Error(String(error));
  if (reason instanceof HostConfigNotFoundError) {
    console.error(colors.red(reason.message));
    console.log();
    try {
      await runWizard({
        detectedHostname: reason.hostname,
        interactiveToggles: true,
      });
    } catch (wizardError) {
      const wizardReason = wizardError instanceof Error
        ? wizardError
        : new Error(String(wizardError));
      console.error(
        colors.red(
          `Failed to launch provisioning wizard: ${wizardReason.message}`,
        ),
      );
      if (wizardReason.stack) {
        console.error(colors.dim(wizardReason.stack));
      }
      Deno.exit(1);
    }
    return;
  }

  console.error(colors.red(`Unexpected error: ${reason.message}`));
  if (reason.stack) {
    console.error(colors.dim(reason.stack));
  }
  Deno.exit(1);
}

if (import.meta.main) {
  try {
    await main();
  } catch (error) {
    await handleTopLevelError(error);
  }
}
