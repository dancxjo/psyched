import { colors } from "$cliffy/ansi/colors.ts";
import { provisionHost, type ProvisionHostOptions } from "./host.ts";
import { repoRoot } from "./paths.ts";
import { resetWorkspace } from "./workspace.ts";

export interface SetupWorkflowOptions {
  host?: string;
  verbose?: boolean;
  skipModules?: boolean;
  skipServices?: boolean;
}

export interface TeardownWorkflowOptions {
  skipModules?: boolean;
  skipServices?: boolean;
  skipClean?: boolean;
}

type HostProvisioner = (
  hostname: string | undefined,
  options: ProvisionHostOptions,
) => Promise<void>;

type PshInvoker = (args: string[]) => Promise<void>;

type WorkspaceCleaner = () => Promise<void>;

let hostProvisioner: HostProvisioner = async (
  hostname: string | undefined,
  options: ProvisionHostOptions,
) => {
  await provisionHost(hostname, options);
};

let pshInvoker: PshInvoker = async (args: string[]) => {
  await runPshSubcommandInBash(args);
};

let workspaceCleaner: WorkspaceCleaner = async () => {
  await resetWorkspace();
};

function shellQuote(value: string): string {
  return `'${value.replace(/'/g, `'"'"'`)}'`;
}

function buildPshInvocation(args: string[]): string {
  const quoted = ["psh", ...args].map(shellQuote);
  return quoted.join(" ");
}

async function runPshSubcommandInBash(args: string[]): Promise<void> {
  if (!args.length) return;
  const commandLine = buildPshInvocation(args);
  const script = `set -euo pipefail\n${commandLine}`;
  const process = new Deno.Command("bash", {
    args: ["-lc", script],
    cwd: repoRoot(),
    stdout: "inherit",
    stderr: "inherit",
  }).spawn();
  const status = await process.status;
  if (!status.success) {
    const code = status.code ?? "unknown";
    const signal = status.signal ?? "";
    const suffix = signal ? ` (signal ${signal})` : "";
    throw new Error(
      `Command '${commandLine}' failed with exit code ${code}${suffix}.`,
    );
  }
}

/**
 * Provision the detected host and configure modules/services using the
 * existing `psh` subcommands inside a fresh Bash login shell.
 *
 * @example
 * ```ts
 * await runSetupWorkflow();
 * await runSetupWorkflow({ host: "pete" });
 * ```
 */
export async function runSetupWorkflow(
  options: SetupWorkflowOptions = {},
): Promise<void> {
  const { host, verbose, skipModules, skipServices } = options;
  const provisionOptions: ProvisionHostOptions = {
    verbose,
    includeModules: false,
    includeServices: false,
  };

  console.log(colors.cyan("==> Provisioning host"));
  await hostProvisioner(host, provisionOptions);

  if (!skipModules) {
    console.log(colors.cyan("==> Configuring modules via 'psh mod setup'"));
    await pshInvoker(["mod", "setup"]);
  } else {
    console.log(colors.yellow("Skipping module setup stage"));
  }

  if (!skipServices) {
    console.log(colors.cyan("==> Configuring services via 'psh srv setup'"));
    await pshInvoker(["srv", "setup"]);
  } else {
    console.log(colors.yellow("Skipping service setup stage"));
  }
}

/**
 * Tear down services and modules, then reset the ROS workspace to a
 * pristine state using `tools/clean_workspace` when available.
 *
 * @example
 * ```ts
 * await runTeardownWorkflow();
 * await runTeardownWorkflow({ skipClean: true });
 * ```
 */
export async function runTeardownWorkflow(
  options: TeardownWorkflowOptions = {},
): Promise<void> {
  const { skipModules, skipServices, skipClean } = options;

  if (!skipServices) {
    console.log(
      colors.cyan("==> Tearing down services via 'psh srv teardown'"),
    );
    await pshInvoker(["srv", "teardown"]);
  } else {
    console.log(colors.yellow("Skipping service teardown stage"));
  }

  if (!skipModules) {
    console.log(colors.cyan("==> Tearing down modules via 'psh mod teardown'"));
    await pshInvoker(["mod", "teardown"]);
  } else {
    console.log(colors.yellow("Skipping module teardown stage"));
  }

  if (!skipClean) {
    console.log(colors.cyan("==> Resetting ROS workspace"));
    await workspaceCleaner();
  } else {
    console.log(colors.yellow("Skipping workspace cleanup"));
  }
}

function resetInternals(): void {
  hostProvisioner = async (hostname, options) => {
    await provisionHost(hostname, options);
  };
  pshInvoker = async (args) => {
    await runPshSubcommandInBash(args);
  };
  workspaceCleaner = async () => {
    await resetWorkspace();
  };
}

export const __test__ = {
  replaceHostProvisioner(fn: HostProvisioner): void {
    hostProvisioner = fn;
  },
  replacePshInvoker(fn: PshInvoker): void {
    pshInvoker = fn;
  },
  replaceWorkspaceCleaner(fn: WorkspaceCleaner): void {
    workspaceCleaner = fn;
  },
  reset(): void {
    resetInternals();
  },
};

resetInternals();
