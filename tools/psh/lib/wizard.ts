import { colors } from "$cliffy/ansi/colors.ts";
import { Checkbox, Confirm, Select } from "$cliffy/prompt/mod.ts";
import {
  availableHosts,
  loadHostConfig,
  provisionHost,
  provisionHostProfile,
} from "./host.ts";
import type { HostConfig, ModuleDirective, ServiceDirective } from "./host.ts";

interface WizardOptions {
  detectedHostname?: string;
  interactiveToggles?: boolean;
}

type ModuleToggle = { setup: boolean; launch: boolean };
type ServiceToggle = { setup: boolean; up: boolean };

function collectModuleToggles(
  modules: ModuleDirective[] | undefined,
  setupSelection: Set<string>,
  launchSelection: Set<string>,
): Map<string, ModuleToggle> {
  const toggles = new Map<string, ModuleToggle>();
  if (!modules?.length) return toggles;
  for (const module of modules) {
    const setup = setupSelection.has(module.name);
    const launch = launchSelection.has(module.name);
    if (!setup && !launch) continue;
    toggles.set(module.name, { setup, launch });
  }
  return toggles;
}

function collectServiceToggles(
  services: ServiceDirective[] | undefined,
  setupSelection: Set<string>,
  upSelection: Set<string>,
): Map<string, ServiceToggle> {
  const toggles = new Map<string, ServiceToggle>();
  if (!services?.length) return toggles;
  for (const service of services) {
    const setup = setupSelection.has(service.name);
    const up = upSelection.has(service.name);
    if (!setup && !up) continue;
    toggles.set(service.name, { setup, up });
  }
  return toggles;
}

function cloneConfig(config: HostConfig): HostConfig {
  if (typeof structuredClone === "function") {
    return structuredClone(config);
  }
  return JSON.parse(JSON.stringify(config)) as HostConfig;
}

function applyModuleToggles(
  modules: ModuleDirective[] | undefined,
  toggles: Map<string, ModuleToggle>,
): ModuleDirective[] {
  if (!modules?.length) return [];
  const next: ModuleDirective[] = [];
  for (const module of modules) {
    const toggle = toggles.get(module.name);
    if (!toggle) continue;
    if (!toggle.setup && !toggle.launch) continue;
    next.push({
      ...module,
      setup: toggle.setup,
      launch: toggle.launch,
    });
  }
  return next;
}

function applyServiceToggles(
  services: ServiceDirective[] | undefined,
  toggles: Map<string, ServiceToggle>,
): ServiceDirective[] {
  if (!services?.length) return [];
  const next: ServiceDirective[] = [];
  for (const service of services) {
    const toggle = toggles.get(service.name);
    if (!toggle) continue;
    if (!toggle.setup && !toggle.up) continue;
    next.push({
      ...service,
      setup: toggle.setup,
      up: toggle.up,
    });
  }
  return next;
}

export async function runWizard(options: WizardOptions = {}): Promise<void> {
  const detectedHostname = options.detectedHostname ?? Deno.hostname();
  const interactiveToggles = options.interactiveToggles ?? false;

  console.log(colors.bold(colors.magenta("Psyched Provisioning Wizard")));
  console.log(
    "Let's prepare this host with the correct modules and services.\n",
  );

  const hosts = availableHosts();
  if (!hosts.length) {
    console.error(colors.red("No host configurations found under hosts/."));
    return;
  }

  const defaultHost = hosts.includes(detectedHostname)
    ? detectedHostname
    : hosts[0];
  const hostname = await Select.prompt<string>({
    message: interactiveToggles
      ? `Select the host profile to apply to '${detectedHostname}'`
      : "Select the host profile to apply",
    options: hosts.map((name) => ({ name, value: name })),
    default: defaultHost,
  });

  const { path: configPath, config: originalConfig } = loadHostConfig(hostname);
  const config = cloneConfig(originalConfig);
  console.log(colors.cyan(`\nHost '${hostname}' provisions:`));
  if (config.modules?.length) {
    console.log(colors.green("  Modules:"));
    for (const module of config.modules) {
      const actions = [module.setup !== false ? "setup" : "skip setup"];
      if (module.launch) actions.push("launch");
      console.log(`    • ${module.name} (${actions.join(", ")})`);
    }
  } else {
    console.log(colors.yellow("  No modules configured."));
  }

  if (config.services?.length) {
    console.log(colors.green("  Services:"));
    for (const service of config.services) {
      const actions = [service.setup !== false ? "setup" : "skip setup"];
      if (service.up) actions.push("start");
      console.log(`    • ${service.name} (${actions.join(", ")})`);
    }
  } else {
    console.log(colors.yellow("  No services configured."));
  }

  if (config.provision?.scripts?.length) {
    console.log(colors.green("  Provisioning scripts:"));
    for (const script of config.provision.scripts) {
      console.log(`    • ${script}`);
    }
  }

  let modules = config.modules ?? [];
  let services = config.services ?? [];

  if (interactiveToggles) {
    console.log();
    if (modules.length) {
      const setupSelection = new Set(
        await Checkbox.prompt<string>({
          message: "Select modules to run setup for",
          options: modules.map((module) => ({
            name: module.name,
            value: module.name,
            checked: module.setup !== false,
          })),
        }),
      );
      const launchSelection = new Set(
        await Checkbox.prompt<string>({
          message: "Select modules to launch after provisioning",
          options: modules.map((module) => ({
            name: module.name,
            value: module.name,
            checked: Boolean(module.launch),
          })),
        }),
      );
      const toggles = collectModuleToggles(
        modules,
        setupSelection,
        launchSelection,
      );
      modules = applyModuleToggles(modules, toggles);
    }

    if (services.length) {
      const setupSelection = new Set(
        await Checkbox.prompt<string>({
          message: "Select services to run setup for",
          options: services.map((service) => ({
            name: service.name,
            value: service.name,
            checked: service.setup !== false,
          })),
        }),
      );
      const upSelection = new Set(
        await Checkbox.prompt<string>({
          message: "Select services to start after provisioning",
          options: services.map((service) => ({
            name: service.name,
            value: service.name,
            checked: Boolean(service.up),
          })),
        }),
      );
      const toggles = collectServiceToggles(
        services,
        setupSelection,
        upSelection,
      );
      services = applyServiceToggles(services, toggles);
    }

    config.modules = modules;
    config.services = services;
  }

  const verbose = await Confirm.prompt({
    message: "Show detailed logs during provisioning?",
    default: false,
  });

  const confirmed = await Confirm.prompt({
    message: `Proceed with provisioning for '${hostname}'?`,
    default: true,
  });

  if (!confirmed) {
    console.log(colors.yellow("Provisioning cancelled."));
    return;
  }

  if (interactiveToggles) {
    await provisionHostProfile({
      detectedHostname,
      profileName: hostname,
      configPath,
      config,
      options: {
        verbose,
        includeModules: config.modules?.length ? true : false,
        includeServices: config.services?.length ? true : false,
      },
    });
  } else {
    await provisionHost(hostname, { verbose });
  }
  console.log(colors.bold(colors.green("\nAll done!")));
  console.log(
    "Next steps: launch everything with `psh up` (or scope it with `psh up <name>`). Use `psh down` to stop targets when you're done.",
  );
}

export const __test__ = {
  applyModuleToggles,
  applyServiceToggles,
};
