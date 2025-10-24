import { join, resolve } from "$std/path/mod.ts";
import { DepGraph } from "$deps/dependency-graph";
import { colors } from "$cliffy/ansi/colors.ts";
import { $ } from "$dax";
import { repoRoot } from "./paths.ts";
import { bringModuleUp, setupModule } from "./module.ts";
import { bringServiceUp, setupService } from "./service.ts";
import { getInstaller, runInstaller } from "./deps/installers.ts";
import { buildRosEnv } from "./ros_env.ts";
import { ensureEssentialUserGroups } from "./user_groups.ts";
import { loadHostConfig, pathExists } from "./host_config.ts";
import type {
  HostConfig,
  ModuleDirective,
  ServiceDirective,
} from "./host_config.ts";

const moduleOps = {
  setup: setupModule,
  launch: bringModuleUp,
};

const serviceOps = {
  setup: setupService,
  start: bringServiceUp,
};

function applyEnv(overrides: Record<string, string> = {}): () => void {
  const previous = new Map<string, string | undefined>();
  for (const [key, value] of Object.entries(overrides)) {
    previous.set(key, Deno.env.get(key));
    Deno.env.set(key, value);
  }
  return () => {
    for (const [key, value] of previous.entries()) {
      if (value === undefined) {
        Deno.env.delete(key);
      } else {
        Deno.env.set(key, value);
      }
    }
  };
}

function resolveScriptPath(script: string, configPath: string): string {
  if (script.startsWith("/")) return script;
  const configDir = resolve(join(configPath, ".."));
  const candidates = [
    script,
    join(configDir, script),
    join(repoRoot(), script),
  ];
  for (const candidate of candidates) {
    const resolved = resolve(candidate);
    if (pathExists(resolved)) return resolved;
  }
  return resolve(join(configDir, script));
}

interface ProvisionRunOptions {
  verbose: boolean;
  showLogsOnSuccess: boolean;
}

export interface ProvisionTask {
  id: string;
  label: string;
  dependencies: string[];
  run: () => Promise<void>;
}

export interface TaskRegistry {
  tasks: ProvisionTask[];
  aliasToId: Map<string, string>;
}

function createTaskRegistry(): TaskRegistry {
  return {
    tasks: [],
    aliasToId: new Map<string, string>(),
  };
}

function registerTask(
  registry: TaskRegistry,
  task: ProvisionTask,
  aliases: string[] = [],
): void {
  if (!registry.aliasToId.has(task.id)) {
    registry.aliasToId.set(task.id, task.id);
  }
  for (const alias of aliases) {
    const existing = registry.aliasToId.get(alias);
    if (existing && existing !== task.id) {
      throw new Error(
        `Alias '${alias}' already registered for task '${existing}'`,
      );
    }
    registry.aliasToId.set(alias, task.id);
  }
  registry.tasks.push(task);
}

function resolveDependencyAliases(
  registry: TaskRegistry,
  task: ProvisionTask,
): void {
  const resolved: string[] = [];
  for (const dependency of task.dependencies) {
    const target = registry.aliasToId.get(dependency) ??
      registry.aliasToId.get(dependency.trim());
    if (!target) {
      throw new Error(
        `Unknown dependency '${dependency}' referenced by task '${task.id}'.`,
      );
    }
    if (target !== task.id && !resolved.includes(target)) {
      resolved.push(target);
    }
  }
  task.dependencies = resolved;
}

export const __test__ = {
  createTaskRegistry,
  registerTask,
  resolveDependencyAliases,
};

export const __internals__ = {
  moduleOps,
  serviceOps,
};

function mergeDependencies(
  defaults: string[],
  extras: string[] | undefined,
): string[] {
  const unique = new Set<string>();
  for (const value of defaults) {
    if (value) unique.add(value);
  }
  if (extras) {
    for (const value of extras) {
      if (value) unique.add(value);
    }
  }
  return Array.from(unique);
}

async function executeProvisionScript(
  configPath: string,
  script: string,
): Promise<void> {
  const resolved = resolveScriptPath(script, configPath);
  if (!pathExists(resolved)) {
    throw new Error(`script not found at ${resolved}`);
  }
  console.log(colors.cyan(`Running ${resolved} …`));
  const result = await $`bash ${resolved}`.stdout("inherit").stderr("inherit")
    .noThrow();
  if (result.code !== 0) {
    throw new Error(`exit ${result.code}`);
  }
}

/** Options that adjust host provisioning behaviour. */
export interface ProvisionHostOptions {
  /** Enable verbose logging for installers and scripts. */
  verbose?: boolean;
  /** Show installer logs even on success when true. */
  showLogsOnSuccess?: boolean;
  /** Include module setup/launch tasks when true. */
  includeModules?: boolean;
  /** Include service setup/start tasks when true. */
  includeServices?: boolean;
}

export async function provisionHost(
  hostname?: string,
  options: ProvisionHostOptions = {},
): Promise<void> {
  const detected = hostname ?? Deno.hostname();
  const { path, config } = loadHostConfig(detected);
  await provisionHostProfile({
    detectedHostname: detected,
    profileName: detected,
    configPath: path,
    config,
    options,
  });
}

export interface ProvisionHostProfileParams {
  detectedHostname: string;
  profileName: string;
  configPath: string;
  config: HostConfig;
  options?: ProvisionHostOptions;
}

export async function provisionHostProfile(
  params: ProvisionHostProfileParams,
): Promise<void> {
  const {
    detectedHostname,
    profileName,
    configPath,
    config,
    options = {},
  } = params;
  console.log(colors.bold(`Detected hostname: ${detectedHostname}`));
  if (profileName !== detectedHostname) {
    console.log(
      colors.bold(
        colors.magenta(`Applying host profile '${profileName}'.`),
      ),
    );
  }
  console.log(colors.cyan(`Loading host config: ${configPath}`));
  const cfg = config;
  const scripts = cfg.provision?.scripts ?? [];
  const hostInstallers = cfg.host.installers ?? [];
  const provisionInstallers = cfg.provision?.installers ?? [];
  const installers = Array.from(
    new Set<string>([...hostInstallers, ...provisionInstallers]),
  );
  const modules: ModuleDirective[] = cfg.modules ?? [];
  const services: ServiceDirective[] = cfg.services ?? [];
  const sharedEnv = buildRosEnv();
  const envVerbose = Deno.env.get("PSH_VERBOSE") === "1";
  const verbose = options.verbose ?? envVerbose;
  const showLogsOnSuccess = options.showLogsOnSuccess ?? verbose;
  const includeModules = options.includeModules ?? false;
  const includeServices = options.includeServices ?? false;
  const provisionOptions: ProvisionRunOptions = { verbose, showLogsOnSuccess };
  const registry = createTaskRegistry();

  registerTask(registry, {
    id: "system:groups",
    label: "Ensure essential user group memberships",
    dependencies: [],
    run: async () => {
      await ensureEssentialUserGroups({
        verbose: provisionOptions.verbose,
        showLogsOnSuccess: provisionOptions.showLogsOnSuccess,
      });
    },
  }, ["groups", "user-groups"]);

  const installerAliasDefaults = installers.map((id) => id);

  for (const id of installers) {
    const taskId = `installer:${id}`;
    registerTask(registry, {
      id: taskId,
      label: `Installer '${id}'`,
      dependencies: [],
      run: async () => {
        const installer = getInstaller(id);
        if (!installer) {
          throw new Error(`Unknown installer '${id}'.`);
        }
        console.log(
          colors.bold(colors.magenta(`\nInstaller: ${installer.label}`)),
        );
        await runInstaller(id, {
          verbose: provisionOptions.verbose,
          showLogsOnSuccess: provisionOptions.showLogsOnSuccess,
        });
      },
    }, [id]);
  }

  for (const script of scripts) {
    const taskId = `script:${script}`;
    registerTask(registry, {
      id: taskId,
      label: `Provision script '${script}'`,
      dependencies: mergeDependencies(installerAliasDefaults, undefined),
      run: async () => {
        await executeProvisionScript(configPath, script);
      },
    }, [script]);
  }

  if (!modules.length) {
    console.log(
      colors.yellow(`No modules configured for host '${profileName}'.`),
    );
  } else if (!includeModules) {
    console.log(
      colors.yellow(
        "Module provisioning skipped during host bootstrap. " +
        "Open a new shell session before running 'psh mod setup' to configure modules.",
      ),
    );
  } else {
    for (const directive of modules) {
      const {
        name: moduleName,
        env: moduleEnvOverrides = {},
        setup = true,
        launch = false,
      } = directive;
      const moduleDefaults = installers.includes("ros2") ? ["ros2"] : [];
      const moduleDependencies = mergeDependencies(
        moduleDefaults,
        directive.depends_on,
      );
      const moduleEnv = { ...sharedEnv, ...moduleEnvOverrides };

      let setupTaskId: string | undefined;
      const moduleAliases = [`module:${moduleName}`, moduleName];
      if (setup) {
        setupTaskId = `module:${moduleName}:setup`;
        registerTask(registry, {
          id: setupTaskId,
          label: `Module '${moduleName}' setup`,
          dependencies: [...moduleDependencies],
          run: async () => {
            const restore = applyEnv(moduleEnv);
            try {
              await moduleOps.setup(moduleName);
              console.log(colors.green(`[${moduleName}] setup complete.`));
            } finally {
              restore();
            }
          },
        }, launch ? [] : moduleAliases);
      }

      if (launch) {
        const launchTaskId = `module:${moduleName}:launch`;
        const launchDeps = setupTaskId
          ? mergeDependencies(moduleDependencies, [setupTaskId])
          : [...moduleDependencies];
        registerTask(registry, {
          id: launchTaskId,
          label: `Module '${moduleName}' launch`,
          dependencies: launchDeps,
          run: async () => {
            const restore = applyEnv(moduleEnv);
            try {
              await moduleOps.launch(moduleName);
            } finally {
              restore();
            }
          },
        }, moduleAliases);
      }
    }
  }

  if (!services.length) {
    console.log(
      colors.yellow(`No services configured for host '${profileName}'.`),
    );
  } else if (!includeServices) {
    console.log(
      colors.yellow(
        "Service provisioning skipped during host bootstrap. " +
        "Run 'psh svc setup' after refreshing your shell if services require setup.",
      ),
    );
  } else {
    for (const directive of services) {
      const {
        name: serviceName,
        env: serviceEnvOverrides = {},
        setup = true,
        up = false,
      } = directive;
      const serviceDefaults = installers.includes("docker") ? ["docker"] : [];
      const serviceDependencies = mergeDependencies(
        serviceDefaults,
        directive.depends_on,
      );
      const serviceEnv = { ...sharedEnv, ...serviceEnvOverrides };

      let setupTaskId: string | undefined;
      const serviceAliases = [`service:${serviceName}`, serviceName];
      if (setup) {
        setupTaskId = `service:${serviceName}:setup`;
        registerTask(registry, {
          id: setupTaskId,
          label: `Service '${serviceName}' setup`,
          dependencies: [...serviceDependencies],
          run: async () => {
            const restore = applyEnv(serviceEnv);
            try {
              await serviceOps.setup(serviceName);
              console.log(colors.green(`[${serviceName}] setup complete.`));
            } finally {
              restore();
            }
          },
        }, up ? [] : serviceAliases);
      }

      if (up) {
        const startTaskId = `service:${serviceName}:start`;
        const startDeps = setupTaskId
          ? mergeDependencies(serviceDependencies, [setupTaskId])
          : [...serviceDependencies];
        registerTask(registry, {
          id: startTaskId,
          label: `Service '${serviceName}' start`,
          dependencies: startDeps,
          run: async () => {
            const restore = applyEnv(serviceEnv);
            try {
              await serviceOps.start(serviceName);
            } finally {
              restore();
            }
          },
        }, serviceAliases);
      }
    }
  }

  for (const task of registry.tasks) {
    resolveDependencyAliases(registry, task);
  }

  const graph = new DepGraph<ProvisionTask>();
  const taskMap = new Map<string, ProvisionTask>();
  for (const task of registry.tasks) {
    if (!graph.hasNode(task.id)) {
      graph.addNode(task.id, task);
    }
    taskMap.set(task.id, task);
  }
  for (const task of registry.tasks) {
    for (const dependency of task.dependencies) {
      graph.addDependency(task.id, dependency);
    }
  }

  let orderedIds: string[];
  try {
    orderedIds = graph.overallOrder();
  } catch (error) {
    const message = error instanceof Error ? error.message : String(error);
    throw new Error(`Task dependency cycle detected: ${message}`);
  }

  const failedTasks = new Set<string>();
  const issues: string[] = [];

  for (const taskId of orderedIds) {
    const task = taskMap.get(taskId);
    if (!task) continue;
    const blockingDeps = task.dependencies.filter((dep) =>
      failedTasks.has(dep)
    );
    if (blockingDeps.length) {
      const blockedLabels = blockingDeps.map((dep) =>
        taskMap.get(dep)?.label ?? dep
      );
      console.warn(
        colors.yellow(
          `${task.label} skipped due to failed dependency (${blockedLabels.join(", ")}).`,
        ),
      );
      issues.push(
        `${task.label} (skipped: ${blockedLabels.join(", ")})`,
      );
      failedTasks.add(taskId);
      continue;
    }

    console.log(colors.bold(`\n→ ${task.label}`));
    try {
      await task.run();
      console.log(colors.green(`✓ ${task.label}`));
    } catch (error) {
      const message = error instanceof Error ? error.message : String(error);
      console.error(colors.red(`✗ ${task.label}: ${message}`));
      issues.push(`${task.label} (${message})`);
      failedTasks.add(taskId);
    }
  }

  if (issues.length) {
    console.warn(colors.yellow(`Host provisioning finished with issues:`));
    for (const issue of issues) {
      console.warn(colors.yellow(`  - ${issue}`));
    }
  } else {
    console.log(
      colors.green(`Host provisioning complete for '${profileName}'.`),
    );
  }
}

export {
  availableHosts,
  locateHostConfig,
  loadHostConfig,
  readHostConfig,
  HostConfigFormatError,
  HostConfigNotFoundError,
} from "./host_config.ts";

export type {
  HostConfig,
  HostConfigEntry,
  HostConfigScopes,
  ModuleDirective,
  ModuleLaunchConfig,
  ServiceDirective,
} from "./host_config.ts";
