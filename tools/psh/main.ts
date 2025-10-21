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
  loadModuleApiActions,
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
  moduleTargetResolution,
  resolveModuleTargets,
  resolveServiceTargets,
  serviceTargetResolution,
} from "./lib/host_targets.ts";
import { resolveTargetBatches } from "./lib/target_resolver.ts";
import {
  debugServiceSystemd,
  debugSystemd,
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

const STREAM_TOPIC_SCHEMA = {
  type: "object",
  properties: {
    topic: {
      type: "string",
      description: "ROS topic name to bridge",
    },
    message_type: {
      type: "string",
      description: "Fully-qualified ROS message type for the topic",
    },
    role: {
      type: "string",
      enum: ["subscribe", "publish", "both"],
      default: "subscribe",
    },
    queue_length: {
      type: "integer",
      minimum: 1,
      default: 10,
    },
    qos: {
      type: "object",
      description: "Optional QoS overrides (reliability, durability)",
      additionalProperties: true,
    },
  },
  required: ["topic", "message_type"],
  additionalProperties: false,
};

const STREAM_TOPIC_RETURNS = {
  type: "object",
  properties: {
    stream: {
      type: "object",
      properties: {
        id: { type: "string" },
        module: { type: "string" },
        topic: { type: "string" },
        message_type: { type: "string" },
        role: { type: "string" },
      },
      required: ["id", "module", "topic", "message_type", "role"],
      additionalProperties: false,
    },
  },
  required: ["stream"],
  additionalProperties: false,
};

const CALL_SERVICE_SCHEMA = {
  type: "object",
  properties: {
    service: {
      type: "string",
      description: "ROS service name to invoke",
    },
    service_type: {
      type: "string",
      description: "Optional service type hint",
    },
    arguments: {
      type: "object",
      description: "Service request arguments",
      additionalProperties: true,
    },
    timeout_ms: {
      type: "integer",
      description: "Optional timeout for the service call in milliseconds",
      minimum: 1,
      default: 8000,
    },
  },
  required: ["service"],
  additionalProperties: false,
};

const CALL_SERVICE_RETURNS = {
  type: "object",
  properties: {
    result: {
      type: "object",
      additionalProperties: true,
    },
  },
  required: ["result"],
  additionalProperties: false,
};

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

  async function runActionsExport(json?: boolean): Promise<void> {
    const cockpit = (Deno.env.get("COCKPIT_URL") || "http://127.0.0.1:8088")
      .replace(/\/$/, "");
    try {
      const resp = await fetch(`${cockpit}/api/actions`, { method: "GET" });
      if (resp.ok) {
        const data = await resp.json();
        if (json) {
          console.log(JSON.stringify(data));
          return;
        }

        const modules = data && typeof data === "object"
          ? (data as Record<string, unknown>).modules || {}
          : {};
        for (const [moduleName, info] of Object.entries(modules)) {
          console.log(`Module: ${moduleName}`);
          const infoRecord = info as Record<string, unknown> | undefined;
          const actions = infoRecord && Array.isArray(infoRecord.actions)
            ? infoRecord.actions
            : [];
          for (const a of actions) {
            const aRec = a as Record<string, unknown> | undefined;
            const name = aRec && typeof aRec.name === "string"
              ? aRec.name
              : "<unknown>";
            const desc = aRec && typeof aRec.description === "string"
              ? aRec.description
              : "";
            console.log(`  - ${name}: ${desc}`);
          }
        }
        return;
      }

      console.warn(
        `Cockpit not responding (${resp.status}); falling back to local export`,
      );
    } catch (err) {
      console.warn(
        "Cockpit API unreachable, falling back to local export:",
        err instanceof Error ? err.message : String(err),
      );
    }

    const moduleNames = listModules();
    const apiActions = loadModuleApiActions();
    const modulesPayload: Record<
      string,
      { actions: Record<string, unknown>[] }
    > = {};

    for (const moduleName of moduleNames) {
      const actions: Record<string, unknown>[] = [];
      const defined = apiActions[moduleName] ?? [];
      const definedNames = new Set<string>();

      for (const action of defined) {
        definedNames.add(action.name);
        const streaming = typeof action.kind === "string"
          ? action.kind.toLowerCase() === "stream-topic"
          : false;
        actions.push({
          name: action.name,
          description: action.description ?? "",
          parameters: action.parameters ?? {},
          returns: action.returns ?? undefined,
          streaming,
        });
      }

      if (!definedNames.has("stream_topic")) {
        actions.push({
          name: "stream_topic",
          description: `Stream ROS topic traffic for the ${moduleName} module`,
          parameters: STREAM_TOPIC_SCHEMA,
          returns: STREAM_TOPIC_RETURNS,
          streaming: true,
        });
      }

      if (!definedNames.has("call_service")) {
        actions.push({
          name: "call_service",
          description:
            `Invoke ROS services on behalf of the ${moduleName} module`,
          parameters: CALL_SERVICE_SCHEMA,
          returns: CALL_SERVICE_RETURNS,
          streaming: false,
        });
      }

      modulesPayload[moduleName] = { actions };
    }

    if (json) {
      console.log(JSON.stringify({ modules: modulesPayload }));
      return;
    }

    for (const [moduleName, info] of Object.entries(modulesPayload)) {
      console.log(`Module: ${moduleName}`);
      for (const action of info.actions) {
        const name = typeof action.name === "string"
          ? action.name
          : "<unknown>";
        const description = typeof action.description === "string"
          ? action.description
          : "";
        console.log(`  - ${name}: ${description}`);
      }
    }
  }

  const actionsCommand = new Command()
    .description("Interact with the cockpit action registry (export|list)")
    .action(function () {
      this.showHelp();
    });

  actionsCommand
    .command("export")
    .description(
      "Fetch actions from the cockpit API (fallback to local definitions)",
    )
    .option("--json", "Emit machine-readable JSON output")
    .action(async ({ json }: { json?: boolean }) => {
      await runActionsExport(Boolean(json));
    });

  actionsCommand
    .command("list")
    .description("Alias for 'export'")
    .option("--json", "Emit machine-readable JSON output")
    .action(async ({ json }: { json?: boolean }) => {
      await runActionsExport(Boolean(json));
    });

  actionsCommand
    .command("help")
    .description("Show help for actions subcommands")
    .action(function () {
      this.parent?.showHelp();
    });

  root.command("actions", actionsCommand);

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
          `- ${status.name}: ${state}${
            status.description ? ` – ${status.description}` : ""
          }`,
        );
      }
    });

  serviceCommand
    .command("setup [services...:string]")
    .description("Run setup for services")
    .option(
      "-b, --build",
      "Rebuild Docker images for the selected services",
    )
    .action(async (options: { build?: boolean }, ...services: string[]) => {
      const targets = resolveServiceTargets("setup", services);
      await setupServices(targets, { build: options.build ?? false });
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

  systemCommand
    .command("setup [targets...:string]")
    .description("Generate systemd units for modules and services")
    .option("--service", "Operate on services only")
    .option("--module", "Operate on modules only")
    .action(async (options: SystemdTargetOptions, ...targets: string[]) => {
      const resolved = resolveSystemdTargets(targets, options);
      for (const target of resolved) {
        if (target.kind === "module") {
          await setupSystemd(target.name);
        } else {
          await setupServiceSystemd(target.name);
        }
      }
    });

  systemCommand
    .command("teardown [targets...:string]")
    .description("Remove systemd units for modules and services")
    .option("--service", "Operate on services only")
    .option("--module", "Operate on modules only")
    .action(async (options: SystemdTargetOptions, ...targets: string[]) => {
      const resolved = resolveSystemdTargets(targets, options);
      for (const target of resolved) {
        if (target.kind === "module") {
          await teardownSystemd(target.name);
        } else {
          await teardownServiceSystemd(target.name);
        }
      }
    });

  systemCommand
    .command("enable [targets...:string]")
    .description("Enable systemd units for modules and services")
    .option("--service", "Operate on services only")
    .option("--module", "Operate on modules only")
    .action(async (options: SystemdTargetOptions, ...targets: string[]) => {
      const resolved = resolveSystemdTargets(targets, options);
      for (const target of resolved) {
        if (target.kind === "module") {
          await enableSystemd(target.name);
        } else {
          await enableServiceSystemd(target.name);
        }
      }
    });

  systemCommand
    .command("disable [targets...:string]")
    .description("Disable systemd units for modules and services")
    .option("--service", "Operate on services only")
    .option("--module", "Operate on modules only")
    .action(async (options: SystemdTargetOptions, ...targets: string[]) => {
      const resolved = resolveSystemdTargets(targets, options);
      for (const target of resolved) {
        if (target.kind === "module") {
          await disableSystemd(target.name);
        } else {
          await disableServiceSystemd(target.name);
        }
      }
    });

  systemCommand
    .command("up [targets...:string]")
    .description("Start modules and services via systemd")
    .option("--service", "Operate on services only")
    .option("--module", "Operate on modules only")
    .action(async (options: SystemdTargetOptions, ...targets: string[]) => {
      const resolved = resolveSystemdTargets(targets, options);
      for (const target of resolved) {
        if (target.kind === "module") {
          await startSystemd(target.name);
        } else {
          await startServiceSystemd(target.name);
        }
      }
    });

  systemCommand
    .command("down [targets...:string]")
    .description("Stop modules and services via systemd")
    .option("--service", "Operate on services only")
    .option("--module", "Operate on modules only")
    .action(async (options: SystemdTargetOptions, ...targets: string[]) => {
      const resolved = resolveSystemdTargets(targets, options);
      for (const target of resolved) {
        if (target.kind === "module") {
          await stopSystemd(target.name);
        } else {
          await stopServiceSystemd(target.name);
        }
      }
    });

  systemCommand
    .command("debug [targets...:string]")
    .description("Show status and recent logs for modules and services")
    .option("--service", "Operate on services only")
    .option("--module", "Operate on modules only")
    .action(async (options: SystemdTargetOptions, ...targets: string[]) => {
      const resolved = resolveSystemdTargets(targets, options);
      for (const target of resolved) {
        if (target.kind === "module") {
          await debugSystemd(target.name);
        } else {
          await debugServiceSystemd(target.name);
        }
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

interface ResolvedSystemdTarget {
  kind: SystemdTargetKind;
  name: string;
}

function resolveSystemdTargets(
  targets: string[] | undefined,
  options: SystemdTargetOptions = {},
): ResolvedSystemdTarget[] {
  if (options.service && options.module) {
    throw new Error("Cannot specify both --service and --module flags.");
  }

  const modules = listModules();
  const services = listServices();
  const catalog = { modules, services };
  const moduleDefaults = moduleTargetResolution("launch").targets;
  const serviceDefaults = serviceTargetResolution("up").targets;

  const resolved: ResolvedSystemdTarget[] = [];
  const seen = new Set<string>();
  const preferService = Boolean(options.service && !options.module);

  let moduleDefaultsApplied = false;
  let serviceDefaultsApplied = false;

  const pushTarget = (kind: SystemdTargetKind, name: string) => {
    const key = `${kind}:${name}`;
    if (seen.has(key)) return;
    seen.add(key);
    resolved.push({ kind, name });
  };

  const tokens = targets?.length ? targets : ["*"];
  for (const token of tokens) {
    if (isWildcardToken(token)) {
      if (!options.service && !moduleDefaultsApplied) {
        for (const name of moduleDefaults) pushTarget("module", name);
        moduleDefaultsApplied = true;
      }
      if (!options.module && !serviceDefaultsApplied) {
        for (const name of serviceDefaults) pushTarget("service", name);
        serviceDefaultsApplied = true;
      }
      continue;
    }

    const matches = resolveTargetBatches([token], {
      catalog,
      defaults: { modules: [], services: [] },
      preferService,
    });

    const matchedModule = matches.modules[0];
    const matchedService = matches.services[0];

    if (options.module) {
      if (matchedModule) {
        pushTarget("module", matchedModule);
      } else if (matchedService) {
        throw new Error(
          `Target '${token}' is not a module. Use --service to manage services.`,
        );
      }
      continue;
    }

    if (options.service) {
      if (matchedService) {
        pushTarget("service", matchedService);
      } else if (matchedModule) {
        throw new Error(
          `Target '${token}' is not a service. Use --module to manage modules.`,
        );
      }
      continue;
    }

    if (matchedModule) {
      pushTarget("module", matchedModule);
      continue;
    }

    if (matchedService) {
      pushTarget("service", matchedService);
      continue;
    }
  }

  if (!resolved.length) {
    throw new Error(
      "No modules or services matched. Provide explicit targets or configure host directives.",
    );
  }

  return resolved;
}

function isWildcardToken(token: string): boolean {
  return token === "*" || token.toLowerCase() === "all";
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
