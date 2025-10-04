/**
 * Utilities for mapping CLI target arguments to module and service batches.
 */
import { listModules } from "./module.ts";
import { listServices } from "./service.ts";

export interface TargetCatalog {
  modules: string[];
  services: string[];
}

export interface ResolveOptions {
  /**
   * Treat ambiguous names (present in both modules and services) as services.
   */
  preferService?: boolean;
  /**
   * Override the discovered module/service names for testing.
   */
  catalog?: TargetCatalog;
}

export interface TargetBatches {
  modules: string[];
  services: string[];
}

/**
 * Split the provided target names into module and service batches.
 *
 * When `targets` is empty the helper returns all known modules and services.
 * Ambiguous names default to modules unless {@link ResolveOptions.preferService}
 * is true.
 *
 * @throws Error when a target does not match any known module or service.
 */
export function resolveTargetBatches(
  targets: string[],
  options: ResolveOptions = {},
): TargetBatches {
  const catalog = options.catalog ?? {
    modules: listModules(),
    services: listServices(),
  };

  const modules = [...catalog.modules];
  const services = [...catalog.services];

  if (!targets.length) {
    return {
      modules,
      services,
    };
  }

  const moduleSet = new Set(modules);
  const serviceSet = new Set(services);
  const resolvedModules: string[] = [];
  const resolvedServices: string[] = [];

  for (const target of targets) {
    const isModule = moduleSet.has(target);
    const isService = serviceSet.has(target);

    if (isModule && isService) {
      if (options.preferService) {
        pushUnique(resolvedServices, target);
      } else {
        pushUnique(resolvedModules, target);
      }
      continue;
    }

    if (isModule) {
      pushUnique(resolvedModules, target);
      continue;
    }

    if (isService) {
      pushUnique(resolvedServices, target);
      continue;
    }

    const knownModules = modules.length ? modules.join(", ") : "<none>";
    const knownServices = services.length ? services.join(", ") : "<none>";
    throw new Error(
      `Unknown target '${target}'. Known modules: ${knownModules}. Known services: ${knownServices}.`,
    );
  }

  return {
    modules: resolvedModules,
    services: resolvedServices,
  };
}

function pushUnique(targets: string[], value: string): void {
  if (!targets.includes(value)) targets.push(value);
}
