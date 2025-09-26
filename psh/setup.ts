// --- Utility Imports & Helpers ---
import { $ } from "./util.ts";
import { parse as parseToml } from "@std/toml";
import { join } from "@std/path";
import {
  applyModuleActions,
  cleanupModuleContext,
  loadModuleSpec,
  prepareModuleContext,
  repoDirFromModules,
} from "./modules.ts";
import {
  runInstallDocker,
  runInstallRos2,
  isRos2Installed,
  isDockerInstalled,
} from "./install.ts";

type ModuleConfigTable = Record<string, Record<string, unknown>>;

async function determineHostName(): Promise<string> {
  const hn = await $`hostname -s`.stdout("piped").stderr("piped");
  const hostname = (hn.stdout || hn.stderr || "").toString().trim();
  return hostname || Deno.env.get("HOST") || "";
}

async function readHostSpec(host: string): Promise<Record<string, unknown>> {
  const repoDir = repoDirFromModules();
  const tomlPath = join(repoDir, "hosts", `${host}.toml`);
  try {
    const text = await Deno.readTextFile(tomlPath);
    return parseToml(text) as Record<string, unknown>;
  } catch (err) {
    if (err instanceof Deno.errors.NotFound) {
      throw new Error(`Host specification not found at ${tomlPath}`);
    }
    throw new Error(`Failed to read host spec ${tomlPath}: ${String(err)}`);
  }
}

async function runModuleSetup(
  moduleName: string,
  config: Record<string, unknown> | null,
): Promise<void> {
  const specInfo = await loadModuleSpec(moduleName);
  if (!specInfo) {
    console.warn(`No module specification found for '${moduleName}'.`);
    return;
  }
  const ctx = await prepareModuleContext(moduleName, config);
  console.log(`Running module actions: ${moduleName}`);
  try {
    await applyModuleActions(specInfo.spec.actions, ctx);
  } finally {
    await cleanupModuleContext(ctx);
  }
}

async function ensureHostModuleSymlink(
  host: string,
  moduleName: string,
): Promise<void> {
  const repoDir = repoDirFromModules();
  const hostModulesDir = join(repoDir, "hosts", host, "modules");
  await Deno.mkdir(hostModulesDir, { recursive: true });
  const linkPath = join(hostModulesDir, moduleName);
  const target = join(repoDir, "modules", moduleName);
  try {
    const info = await Deno.lstat(linkPath);
    if (info.isSymlink) {
      const existing = await Deno.readLink(linkPath).catch(() => null);
      if (existing === target) return;
    }
    await Deno.remove(linkPath, { recursive: true });
  } catch (err) {
    if (!(err instanceof Deno.errors.NotFound)) throw err;
  }
  await Deno.symlink(target, linkPath);
}

function extractModuleList(spec: Record<string, unknown>): string[] {
  const modules = spec.modules;
  if (!modules) return [];
  if (Array.isArray(modules)) {
    return modules.map((m) => String(m));
  }
  return [String(modules)];
}

function extractModuleConfigs(
  spec: Record<string, unknown>,
): ModuleConfigTable {
  const table = spec.module_configs && typeof spec.module_configs === "object"
    ? spec.module_configs as Record<string, unknown>
    : {};
  const configs: ModuleConfigTable = {};
  for (const [moduleName, cfg] of Object.entries(table)) {
    if (cfg && typeof cfg === "object") {
      configs[moduleName] = cfg as Record<string, unknown>;
    }
  }
  return configs;
}

async function applyHost(host: string): Promise<void> {
  const spec = await readHostSpec(host);
  console.log(`Applying host '${host}'`);

  if (spec.setup_ros2) {
    console.log("Host requests ROS2 installation.");
    if (await isRos2Installed()) {
      console.log("ROS2 already detected on system — skipping installation.");
    } else {
      await runInstallRos2();
    }
  }
  if (spec.setup_docker) {
    console.log("Host requests Docker installation.");
    if (await isDockerInstalled()) {
      console.log("Docker already detected on system — skipping installation.");
    } else {
      await runInstallDocker();
    }
  }

  const modules = extractModuleList(spec);
  if (!modules.length) {
    console.log("No modules listed for this host in TOML. Nothing more to do.");
    return;
  }

  const configs = extractModuleConfigs(spec);
  for (const moduleName of modules) {
    await ensureHostModuleSymlink(host, moduleName);
    const moduleConfig = configs[moduleName] ?? null;
    try {
      await runModuleSetup(moduleName, moduleConfig);
    } catch (err) {
      console.error(`Failed to setup module ${moduleName}:`, String(err));
    }
  }

  console.log("Host setup finished.");
}

export async function setupHosts(hosts: string[]): Promise<void> {
  const targets = hosts.length ? hosts : [await determineHostName()];
  if (!targets[0]) {
    throw new Error(
      "Unable to determine host name. Provide it as `psh provision <host>` or set HOST env var.",
    );
  }
  for (const host of targets) {
    await applyHost(host);
  }
}

/**
 * Return the list of modules for the provided host. If host is not provided,
 * the function will attempt to determine the current hostname.
 */
export async function getHostModules(host?: string): Promise<string[]> {
  const target = host && host.length ? host : await determineHostName();
  if (!target) return [];
  try {
    const spec = await readHostSpec(target);
    return extractModuleList(spec);
  } catch (_err) {
    return [];
  }
}
