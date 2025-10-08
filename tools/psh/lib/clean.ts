import { colors } from "$cliffy/ansi/colors.ts";
import { listModules, teardownModules } from "./module.ts";
import { listServices, teardownServices } from "./service.ts";
import { resetWorkspace } from "./workspace.ts";

const PROTECTED_MODULES = new Set(["wifi", "ssh", "mdns"]);
const PROTECTED_SERVICES = new Set(["ssh", "mdns"]);

export interface CleanOptions {
  skipModules?: boolean;
  skipServices?: boolean;
  skipWorkspace?: boolean;
}

type ModuleLister = () => string[];
type ModuleTeardown = (modules: string[]) => Promise<void>;
type ServiceLister = () => string[];
type ServiceTeardown = (services: string[]) => Promise<void>;
type WorkspaceReset = () => Promise<void>;

let moduleLister: ModuleLister = () => listModules();
let moduleTeardown: ModuleTeardown = (modules) => teardownModules(modules);
let serviceLister: ServiceLister = () => listServices();
let serviceTeardown: ServiceTeardown = (services) => teardownServices(services);
let workspaceReset: WorkspaceReset = () => resetWorkspace();

function partitionProtected(
  items: string[],
  protectedSet: ReadonlySet<string>,
): { teardown: string[]; preserved: string[] } {
  const teardown: string[] = [];
  const preserved: string[] = [];
  for (const item of items) {
    if (protectedSet.has(item)) {
      preserved.push(item);
    } else {
      teardown.push(item);
    }
  }
  return { teardown, preserved };
}

function logPreserved(kind: "module" | "service", names: string[]): void {
  if (!names.length) return;
  const suffix = names.length === 1
    ? `${kind} '${names[0]}'`
    : `${kind}s ${names.join(", ")}`;
  console.log(
    colors.dim(`Preserving protected ${suffix}; skipping teardown.`),
  );
}

function summarizeError(scope: string, error: unknown): string {
  if (error instanceof Error) return `[${scope}] ${error.message}`;
  return `[${scope}] ${String(error)}`;
}

/**
 * Tear down modules, services, and the ROS workspace to restore a clean state.
 *
 * @example
 * ```ts
 * await cleanEnvironment();
 * await cleanEnvironment({ skipWorkspace: true });
 * ```
 */
export async function cleanEnvironment(
  options: CleanOptions = {},
): Promise<void> {
  const { skipModules, skipServices, skipWorkspace } = options;
  const errors: { scope: string; cause: unknown }[] = [];

  if (!skipModules) {
    const modules = moduleLister();
    const { teardown, preserved } = partitionProtected(
      modules,
      PROTECTED_MODULES,
    );
    logPreserved("module", preserved);
    if (teardown.length) {
      console.log(
        colors.cyan(`==> Tearing down ${teardown.length} module(s)`),
      );
      try {
        await moduleTeardown(teardown);
      } catch (error) {
        errors.push({ scope: "modules", cause: error });
        console.error(colors.red(summarizeError("modules", error)));
      }
    } else {
      console.log(colors.dim("No modules to tear down."));
    }
  } else {
    console.log(colors.yellow("Skipping module teardown stage"));
  }

  if (!skipServices) {
    const services = serviceLister();
    const { teardown, preserved } = partitionProtected(
      services,
      PROTECTED_SERVICES,
    );
    logPreserved("service", preserved);
    if (teardown.length) {
      console.log(
        colors.cyan(`==> Tearing down ${teardown.length} service(s)`),
      );
      try {
        await serviceTeardown(teardown);
      } catch (error) {
        errors.push({ scope: "services", cause: error });
        console.error(colors.red(summarizeError("services", error)));
      }
    } else {
      console.log(colors.dim("No services to tear down."));
    }
  } else {
    console.log(colors.yellow("Skipping service teardown stage"));
  }

  if (!skipWorkspace) {
    try {
      console.log(colors.cyan("==> Resetting ROS workspace"));
      await workspaceReset();
    } catch (error) {
      errors.push({ scope: "workspace", cause: error });
      console.error(colors.red(summarizeError("workspace", error)));
    }
  } else {
    console.log(colors.yellow("Skipping workspace reset"));
  }

  if (errors.length) {
    const summary = errors.map((entry) => entry.scope).join(", ");
    throw new AggregateError(
      errors.map((entry) => entry.cause),
      `Failed to complete clean: ${summary}`,
    );
  }
}

function resetInternals(): void {
  moduleLister = () => listModules();
  moduleTeardown = (modules) => teardownModules(modules);
  serviceLister = () => listServices();
  serviceTeardown = (services) => teardownServices(services);
  workspaceReset = () => resetWorkspace();
}

export const __test__ = {
  replaceModuleOps(
    lister?: ModuleLister,
    teardown?: ModuleTeardown,
  ): void {
    if (lister) moduleLister = lister;
    if (teardown) moduleTeardown = teardown;
  },
  replaceServiceOps(
    lister?: ServiceLister,
    teardown?: ServiceTeardown,
  ): void {
    if (lister) serviceLister = lister;
    if (teardown) serviceTeardown = teardown;
  },
  replaceWorkspaceReset(reset?: WorkspaceReset): void {
    if (reset) workspaceReset = reset;
  },
  reset(): void {
    resetInternals();
  },
};
