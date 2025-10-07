import { HostConfigNotFoundError, loadHostConfig } from "./host.ts";
import type { ModuleDirective, ServiceDirective } from "./host.ts";
import { listModules } from "./module.ts";
import { listServices } from "./service.ts";

type ModuleAction = "setup" | "launch" | "teardown";
type ServiceAction = "setup" | "up" | "teardown";

interface HostProfileTargets {
  modules: Map<string, ModuleDirective>;
  services: Map<string, ServiceDirective>;
}

interface TargetResolution {
  targets: string[];
  fromHost: boolean;
}

type ModuleLister = () => string[];
type ServiceLister = () => string[];
type HostCandidateProvider = () => Iterable<string>;

let moduleLister: ModuleLister = () => listModules();
let serviceLister: ServiceLister = () => listServices();
let hostCandidateProvider: HostCandidateProvider = () => {
  const candidates = new Set<string>();
  const override = Deno.env.get("PSH_HOST");
  if (override?.trim()) candidates.add(override.trim());
  try {
    candidates.add(Deno.hostname());
  } catch (_error) {
    // hostname detection can fail; ignore.
  }
  return candidates;
};

let cachedProfile: HostProfileTargets | null | undefined;

function detectHostProfile(): HostProfileTargets | null {
  if (cachedProfile !== undefined) {
    return cachedProfile;
  }

  for (const candidate of hostCandidateProvider()) {
    try {
      const { config } = loadHostConfig(candidate);
      const modules = new Map<string, ModuleDirective>();
      for (const directive of config.modules ?? []) {
        if (!directive?.name) continue;
        modules.set(directive.name, directive);
      }
      const services = new Map<string, ServiceDirective>();
      for (const directive of config.services ?? []) {
        if (!directive?.name) continue;
        services.set(directive.name, directive);
      }
      const profile: HostProfileTargets = { modules, services };
      cachedProfile = profile;
      return profile;
    } catch (error) {
      if (error instanceof HostConfigNotFoundError) continue;
      throw error;
    }
  }

  cachedProfile = null;
  return null;
}

function moduleTargetsFromHost(action: ModuleAction): TargetResolution {
  const available = moduleLister();
  const availableSet = new Set(available);
  const profile = detectHostProfile();
  if (!profile) {
    return { targets: available, fromHost: false };
  }

  const results: string[] = [];
  for (const directive of profile.modules.values()) {
    if (!availableSet.has(directive.name)) continue;
    const setupEnabled = directive.setup !== false;
    const launchEnabled = directive.launch === true;

    let include = false;
    switch (action) {
      case "setup":
        include = setupEnabled;
        break;
      case "launch":
        include = launchEnabled;
        break;
      case "teardown":
        include = setupEnabled || launchEnabled;
        break;
    }
    if (include && !results.includes(directive.name)) {
      results.push(directive.name);
    }
  }
  return { targets: results, fromHost: true };
}

function serviceTargetsFromHost(action: ServiceAction): TargetResolution {
  const available = serviceLister();
  const availableSet = new Set(available);
  const profile = detectHostProfile();
  if (!profile) {
    return { targets: available, fromHost: false };
  }

  const results: string[] = [];
  for (const directive of profile.services.values()) {
    if (!availableSet.has(directive.name)) continue;
    const setupEnabled = directive.setup !== false;
    const upEnabled = directive.up === true;

    let include = false;
    switch (action) {
      case "setup":
        include = setupEnabled;
        break;
      case "up":
        include = upEnabled;
        break;
      case "teardown":
        include = setupEnabled || upEnabled;
        break;
    }
    if (include && !results.includes(directive.name)) {
      results.push(directive.name);
    }
  }
  return { targets: results, fromHost: true };
}

function resolveTargets<TAction extends ModuleAction | ServiceAction>(
  action: TAction,
  explicit: string[] | undefined,
  resolver: (action: TAction) => TargetResolution,
): string[] {
  if (explicit?.length) return explicit;
  const { targets, fromHost } = resolver(action);
  if (!fromHost) return targets;
  return targets;
}

export function resolveModuleTargets(
  action: ModuleAction,
  modules?: string[],
): string[] {
  return resolveTargets(
    action,
    modules,
    moduleTargetsFromHost as (action: ModuleAction) => TargetResolution,
  );
}

export function resolveServiceTargets(
  action: ServiceAction,
  services?: string[],
): string[] {
  return resolveTargets(
    action,
    services,
    serviceTargetsFromHost as (action: ServiceAction) => TargetResolution,
  );
}

export function defaultModuleTargets(action: ModuleAction): string[] {
  return moduleTargetsFromHost(action).targets;
}

export function defaultServiceTargets(action: ServiceAction): string[] {
  return serviceTargetsFromHost(action).targets;
}

export function resetHostTargetCache(): void {
  cachedProfile = undefined;
}

export const __internals__ = {
  setModuleLister(fn: ModuleLister): void {
    moduleLister = fn;
  },
  setServiceLister(fn: ServiceLister): void {
    serviceLister = fn;
  },
  setCandidateProvider(fn: HostCandidateProvider): void {
    hostCandidateProvider = fn;
    cachedProfile = undefined;
  },
  reset(): void {
    moduleLister = () => listModules();
    serviceLister = () => listServices();
    hostCandidateProvider = () => {
      const candidates = new Set<string>();
      const override = Deno.env.get("PSH_HOST");
      if (override?.trim()) candidates.add(override.trim());
      try {
        candidates.add(Deno.hostname());
      } catch (_error) {
        // ignore
      }
      return candidates;
    };
    cachedProfile = undefined;
  },
};

export type { ModuleAction, ServiceAction };
