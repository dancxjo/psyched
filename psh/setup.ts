// --- Utility Imports & Helpers ---
import { $, hasFlag } from "./util.ts";
import { parse as parseToml } from "@std/toml";
import { join } from "@std/path";
import {
  applyModuleActions,
  cleanupModuleContext,
  loadHostModuleConfig,
  loadModuleSpec,
  prepareModuleContext,
  type PrepareModuleContextOptions,
  repoDirFromModules,
} from "./modules.ts";
import {
  isCudaInstalled,
  isDockerInstalled,
  isRos2Installed,
  runInstallCuda,
  runInstallDocker,
  runInstallRos2,
} from "./install.ts";
import {
  isSpeechSetupComplete,
  runSetupSpeech,
} from "./speech_setup.ts";

export interface SetupDeps {
  isRos2Installed(): Promise<boolean> | boolean;
  runInstallRos2(): Promise<void> | void;
  isDockerInstalled(): Promise<boolean> | boolean;
  runInstallDocker(): Promise<void> | void;
  isCudaInstalled(): Promise<boolean> | boolean;
  runInstallCuda(): Promise<void> | void;
  isSpeechSetupComplete(): Promise<boolean> | boolean;
  runSetupSpeech(): Promise<void> | void;
}

const defaultSetupDeps: SetupDeps = {
  isRos2Installed,
  runInstallRos2,
  isDockerInstalled,
  runInstallDocker,
  isCudaInstalled,
  runInstallCuda,
  isSpeechSetupComplete,
  runSetupSpeech,
};

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
  options?: PrepareModuleContextOptions,
): Promise<void> {
  const specInfo = await loadModuleSpec(moduleName);
  if (!specInfo) {
    console.warn(`No module specification found for '${moduleName}'.`);
    return;
  }
  const ctx = await prepareModuleContext(moduleName, options);
  console.log(`Running module actions: ${moduleName}`);
  try {
    await applyModuleActions(specInfo.spec.actions, ctx);
  } finally {
    await cleanupModuleContext(ctx);
  }
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

async function applyHost(host: string, deps: SetupDeps): Promise<void> {
  const spec = await readHostSpec(host);
  console.log(`Applying host '${host}'`);

  const wantsRos2 = hasFlag(spec, "setup_ros2", "setup-ros2");
  if (wantsRos2) {
    console.log("Host requests ROS2 installation.");
    if (await deps.isRos2Installed()) {
      console.log("ROS2 already detected on system — skipping installation.");
    } else {
      await deps.runInstallRos2();
    }
  }
  const wantsDocker = hasFlag(spec, "setup_docker", "setup-docker");
  if (wantsDocker) {
    console.log("Host requests Docker installation.");
    if (await deps.isDockerInstalled()) {
      console.log("Docker already detected on system — skipping installation.");
    } else {
      await deps.runInstallDocker();
    }
  }

  const wantsCuda = hasFlag(spec, "setup_cuda", "setup-cuda");
  if (wantsCuda) {
    console.log("Host requests CUDA installation.");
    if (await deps.isCudaInstalled()) {
      console.log("CUDA toolkit already detected on system — skipping installation.");
    } else {
      await deps.runInstallCuda();
    }
  }

  const wantsSpeech = hasFlag(spec, "setup_speech", "setup-speech");
  if (wantsSpeech) {
    console.log("Host requests speech stack asset setup.");
    if (await deps.isSpeechSetupComplete()) {
      console.log("Speech stack assets already detected — skipping setup.");
    } else {
      await deps.runSetupSpeech();
    }
  }

  const modules = extractModuleList(spec);
  if (!modules.length) {
    console.log("No modules listed for this host in TOML. Nothing more to do.");
    return;
  }

  const configs = extractModuleConfigs(spec);
  for (const moduleName of modules) {
    const inlineConfig = configs[moduleName] ?? null;
    const { path: yamlPath, data: yamlData } = await loadHostModuleConfig(
      host,
      moduleName,
    );

    let configData: Record<string, unknown> | null = yamlData ?? null;
    let configPath: string | null = yamlPath;

    if (!configPath && inlineConfig) {
      configData = inlineConfig;
    } else if (configPath && inlineConfig && Object.keys(inlineConfig).length) {
      console.warn(
        `[setup] host ${host} module_configs.${moduleName} is ignored in favour of YAML at ${configPath}.`,
      );
    }

    try {
      await runModuleSetup(moduleName, {
        configData,
        configPath,
      });
    } catch (err) {
      console.error(`Failed to setup module ${moduleName}:`, String(err));
    }
  }

  console.log("Host setup finished.");
}

export async function setupHosts(
  hosts: string[],
  overrides: Partial<SetupDeps> = {},
): Promise<void> {
  const deps: SetupDeps = { ...defaultSetupDeps, ...overrides } as SetupDeps;
  const targets = hosts.length ? hosts : [await determineHostName()];
  if (!targets[0]) {
    throw new Error(
      "Unable to determine host name. Provide it as `psh provision <host>` or set HOST env var.",
    );
  }
  for (const host of targets) {
    await applyHost(host, deps);
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
