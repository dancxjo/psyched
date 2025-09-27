import { join } from "@std/path";
import { parse as parseToml } from "@std/toml";
import { stringify as stringifyYaml } from "@std/yaml";
import { $ } from "./util.ts";
import {
  loadHostModuleConfig,
  loadModuleSpec,
  ModuleSystemdSpec,
  repoDirFromModules,
} from "./modules.ts";

interface HostSpec {
  setup_ros2?: boolean;
  setup_docker?: boolean;
  modules?: unknown[];
  module_configs?: Record<string, unknown>;
}

/**
 * Escape characters that systemd expands in unit files when using
 * ExecStart= with an inline command. systemd expands $ and % sequences,
 * so replace them with doubled versions so the shell receives the literal
 * characters. This keeps the generated unit stable and prevents systemd
 * from evaluating variables meant for the inline shell script.
 */
function escapeForSystemd(s: string): string {
  // In String.replace the replacement string treats "$" sequences specially
  // (e.g. "$$" -> "$"), so use "$$$$" to insert two literal dollars.
  return s.replace(/\$/g, "$$$$").replace(/%/g, "%%");
}

function ensureArray(value: unknown): string[] {
  if (!value) return [];
  if (Array.isArray(value)) {
    return value.map((v) => String(v));
  }
  return [String(value)];
}

async function readHostSpec(host: string): Promise<HostSpec> {
  const path = `${Deno.cwd()}/hosts/${host}.toml`;
  const text = await Deno.readTextFile(path);
  return parseToml(text) as HostSpec;
}

function normalizeEnvKey(key: string): string {
  return key.replace(/[^A-Za-z0-9]+/g, "_")
    .replace(/^_+|_+$/g, "")
    .toUpperCase();
}

function stringifyEnvValue(value: unknown): string | null {
  if (value === null || value === undefined) return null;
  if (typeof value === "boolean") return value ? "true" : "false";
  if (typeof value === "number" || typeof value === "bigint") {
    return value.toString();
  }
  if (typeof value === "string") return value;
  return null;
}

export function deriveModuleConfigEnv(
  moduleName: string,
  configData: Record<string, unknown> | null | undefined,
): Record<string, string> {
  if (!configData) return {};

  const prefix = normalizeEnvKey(moduleName);
  const env: Record<string, string> = {};

  for (const [rawKey, rawValue] of Object.entries(configData)) {
    const value = stringifyEnvValue(rawValue);
    if (value === null) continue;
    const envKey = normalizeEnvKey(rawKey);
    if (!envKey) continue;
    const scopedKey = prefix ? `${prefix}_${envKey}` : envKey;
    env[scopedKey] = value;
  }

  if (moduleName === "voice") {
    const voicesDir = stringifyEnvValue(configData["voices_dir"]);
    if (voicesDir !== null) {
      env["PIPER_VOICES_DIR"] = voicesDir;
    }
  }

  return env;
}

export function buildUnitContent(
  moduleName: string,
  spec: ModuleSystemdSpec,
  repoDir: string,
  host: string,
  configPath: string | null,
  configData: Record<string, unknown> | null,
): string {
  const description = spec.description ?? `Psyched ${moduleName} Service`;
  const after = spec.after?.length
    ? spec.after.join(" ")
    : "network.target network-online.target";
  const wants = spec.wants?.length
    ? spec.wants.join(" ")
    : "network-online.target";
  const launchCommand = spec.launch_command?.trim() || null;
  const shutdownCommand = spec.shutdown_command?.trim() || null;
  const moduleDir = join(repoDir, "modules", moduleName);
  // Always use absolute path for launch/shutdown scripts unless launch_command/shutdown_command is set
  const defaultLaunchRel = `modules/${moduleName}/${moduleName}.launch.sh`;
  const defaultShutdownRel = `modules/${moduleName}/${moduleName}.shutdown.sh`;
  const launchPath = launchCommand ? null : join(repoDir, defaultLaunchRel);
  const shutdownPath = shutdownCommand
    ? null
    : join(repoDir, defaultShutdownRel);
  const entrypoint = join(repoDir, "tools", "systemd_entrypoint.sh");
  const workingDir = spec.working_directory
    ? join(repoDir, spec.working_directory)
    : repoDir;
  const restart = spec.restart ?? "on-failure";
  const restartSec = spec.restart_sec ?? 2;
  const user = spec.user ?? Deno.env.get("PSH_SYSTEMD_USER") ??
    Deno.env.get("USER") ?? "root";
  const killMode = spec.kill_mode ?? "control-group";
  const timeoutStop = spec.timeout_stop_sec ?? 20;

  if (!launchCommand && !launchPath) {
    throw new Error(
      `[systemd] Module ${moduleName} has no launch command or script defined.`,
    );
  }

  // Expand ${MODULE_DIR} in user-provided commands to absolute module path
  let resolvedLaunchCommand: string | null = launchCommand;
  if (resolvedLaunchCommand) {
    resolvedLaunchCommand = resolvedLaunchCommand.replace(
      /\$\{MODULE_DIR\}/g,
      moduleDir,
    ).replace(/\$MODULE_DIR/g, moduleDir);
  }
  let resolvedShutdownCommand: string | null = shutdownCommand;
  if (resolvedShutdownCommand) {
    resolvedShutdownCommand = resolvedShutdownCommand.replace(
      /\$\{MODULE_DIR\}/g,
      moduleDir,
    ).replace(/\$MODULE_DIR/g, moduleDir);
  }

  // Helper to decide if a command is just a single script path we can pass directly
  function isSinglePath(cmd: string): boolean {
    const stripped = cmd.trim().replace(/^['"]|['"]$/g, "");
    // no whitespace and contains a slash (path-like) or starts with repo/module dir
    return !/\s/.test(stripped) &&
      (stripped.startsWith("/") || stripped.startsWith(moduleDir) ||
        stripped.includes("/"));
  }

  // Compose ExecStart
  let execStart: string | null = null;
  if (resolvedLaunchCommand) {
    if (isSinglePath(resolvedLaunchCommand)) {
      const p = resolvedLaunchCommand.trim().replace(/^['"]|['"]$/g, "");
      execStart = `${entrypoint} ${p}`;
    } else {
      execStart = `${entrypoint} bash -lc ${
        JSON.stringify(escapeForSystemd(resolvedLaunchCommand))
      }`;
    }
  } else if (launchPath) {
    execStart = `${entrypoint} ${launchPath}`;
  }

  // Compose ExecStop
  let execStop: string | null = null;
  if (resolvedShutdownCommand) {
    if (isSinglePath(resolvedShutdownCommand)) {
      const p = resolvedShutdownCommand.trim().replace(/^['"]|['"]$/g, "");
      execStop = `${entrypoint} ${p}`;
    } else {
      execStop = `${entrypoint} bash -lc ${
        JSON.stringify(escapeForSystemd(resolvedShutdownCommand))
      }`;
    }
  } else if (shutdownPath) {
    execStop = `${entrypoint} ${shutdownPath}`;
  }

  const envLines: string[] = [];
  const env = spec.environment ?? {};
  const envEntries = Object.entries(env);
  for (const [key, value] of envEntries) {
    envLines.push(`Environment=${key}=${value}`);
  }

  const configEnv = deriveModuleConfigEnv(moduleName, configData);
  for (const [key, value] of Object.entries(configEnv)) {
    envLines.push(`Environment=${key}=${value}`);
  }
  envLines.push(`Environment=HOST=${host}`);
  envLines.push(`Environment=PSH_MODULE_NAME=${moduleName}`);
  if (configPath) {
    envLines.push(`Environment=PSH_MODULE_CONFIG=${configPath}`);
  }

  const serviceLines = [
    "[Service]",
    "Type=simple",
    `WorkingDirectory=${workingDir}`,
  ];
  if (envLines.length > 0) {
    serviceLines.push(...envLines);
  }
  serviceLines.push(`ExecStart=${execStart}`);
  if (execStop) {
    serviceLines.push(`ExecStop=${execStop}`);
  }
  serviceLines.push(`Restart=${restart}`);
  serviceLines.push(`RestartSec=${restartSec}`);
  serviceLines.push(`User=${user}`);
  serviceLines.push(`KillMode=${killMode}`);
  serviceLines.push(`TimeoutStopSec=${timeoutStop}`);

  return `# Generated by psh systemd
[Unit]
Description=${description}
After=${after}
Wants=${wants}

${serviceLines.join("\n")}

[Install]
WantedBy=multi-user.target
`;
}

async function ensureHostModuleConfig(
  host: string,
  module: string,
  config: Record<string, unknown> | undefined,
): Promise<{ path: string | null; data: Record<string, unknown> | null }> {
  const { path, data } = await loadHostModuleConfig(host, module);
  if (path) {
    if (config && Object.keys(config).length) {
      console.warn(
        `[systemd] host ${host} module_configs.${module} ignored; using YAML at ${path}.`,
      );
    }
    return { path, data };
  }

  if (config && Object.keys(config).length > 0) {
    const repoDir = repoDirFromModules();
    const configDir = join(repoDir, "hosts", host, "config");
    await Deno.mkdir(configDir, { recursive: true });
    const configPath = join(configDir, `${module}.yaml`);
    const yaml = stringifyYaml(config);
    await Deno.writeTextFile(configPath, yaml);
    console.warn(
      `[systemd] Wrote ${configPath} from inline module_configs.${module}; migrate to YAML to silence this.`,
    );
    return { path: configPath, data: config };
  }

  return { path: null, data: null };
}

async function generateUnits(
  host: string,
  unitsFilter?: string[],
): Promise<void> {
  const repoDir = repoDirFromModules();
  const hostSpec = await readHostSpec(host);
  const modulesList = ensureArray(hostSpec.modules);
  const configs = hostSpec.module_configs ?? {};
  const outputDir = join(repoDir, "hosts", host, "systemd");
  await Deno.mkdir(outputDir, { recursive: true });

  const desired = new Set<string>();
  for (const moduleName of modulesList) {
    if (unitsFilter && unitsFilter.length > 0) {
      // unitsFilter contains module names or service names; normalize
      const matches = unitsFilter.some((u) =>
        u === moduleName || u === `psyched-${moduleName}` ||
        u === `psyched-${moduleName}.service`
      );
      if (!matches) continue;
    }
    const moduleSpecInfo = await loadModuleSpec(moduleName);
    if (!moduleSpecInfo?.spec.systemd) {
      console.log(
        `[systemd] Module ${moduleName} has no systemd section; skipping.`,
      );
      continue;
    }
    const moduleConfig = (configs[moduleName] ?? {}) as Record<string, unknown>;
    const { path: configPath, data: configData } = await ensureHostModuleConfig(
      host,
      moduleName,
      moduleConfig,
    );
    const unitContent = buildUnitContent(
      moduleName,
      moduleSpecInfo.spec.systemd,
      repoDir,
      host,
      configPath,
      configData,
    );
    const serviceName = `psyched-${moduleName}.service`;
    const unitPath = join(outputDir, serviceName);
    await Deno.writeTextFile(unitPath, unitContent);
    console.log(`[systemd] Wrote ${unitPath}`);
    desired.add(serviceName);
  }

  for await (const entry of Deno.readDir(outputDir)) {
    if (!entry.isFile || !entry.name.endsWith(".service")) continue;
    if (desired.has(entry.name)) continue;
    const stalePath = join(outputDir, entry.name);
    console.log(`[systemd] Removing stale unit ${stalePath}`);
    await Deno.remove(stalePath).catch(() => undefined);
    await $`sudo systemctl disable --now ${entry.name}`.noThrow();
    const systemdDir = Deno.env.get("SYSTEMD_DIR") ??
      Deno.env.get("PSH_SYSTEMD_DIR") ?? "/etc/systemd/system";
    await $`sudo rm -f ${join(systemdDir, entry.name)}`.noThrow();
  }
}

export async function systemdGenerate(): Promise<void> {
  const hn = await $`hostname -s`.stdout("piped");
  const host = (hn.stdout || hn.stderr || "").toString().trim() ||
    Deno.env.get("HOST") || "default";
  // no units filter by default
  await generateUnits(host);
}

export async function systemdInstall(units?: string[]): Promise<void> {
  // regenerate units for current host, optionally filtered
  const hn = await $`hostname -s`.stdout("piped");
  const host = (hn.stdout || hn.stderr || "").toString().trim() ||
    Deno.env.get("HOST") || "default";
  await generateUnits(host, units);
  const repoDir = repoDirFromModules();
  const unitsDir = join(repoDir, "hosts", host, "systemd");
  const systemdDir = Deno.env.get("SYSTEMD_DIR") ??
    Deno.env.get("PSH_SYSTEMD_DIR") ?? "/etc/systemd/system";
  for await (const ent of Deno.readDir(unitsDir)) {
    if (!ent.isFile || !ent.name.endsWith(".service")) continue;
    const src = join(unitsDir, ent.name);
    const dest = join(systemdDir, ent.name);
    console.log(`[systemd] Installing ${ent.name}`);
    const copy = await $`sudo cp ${src} ${dest}`.noThrow();
    if (copy.code !== 0) {
      console.error(
        `[systemd] Failed to copy ${ent.name}:`,
        copy.stderr || copy.stdout,
      );
      continue;
    }
  }
  await $`sudo systemctl daemon-reload`.noThrow();
  // enable units if requested explicitly via enable command
}

export async function systemdUninstall(): Promise<void> {
  const hn = await $`hostname -s`;
  const host = (hn.stdout || hn.stderr || "").toString().trim() ||
    Deno.env.get("HOST") || "default";
  const repoDir = repoDirFromModules();
  const unitsDir = join(repoDir, "hosts", host, "systemd");
  const systemdDir = Deno.env.get("SYSTEMD_DIR") ??
    Deno.env.get("PSH_SYSTEMD_DIR") ?? "/etc/systemd/system";
  try {
    const stat = await Deno.stat(unitsDir);
    if (!stat.isDirectory) {
      console.log(`[systemd] No generated units in ${unitsDir}`);
      return;
    }
  } catch {
    console.log(`[systemd] No generated units in ${unitsDir}`);
    return;
  }
  for await (const ent of Deno.readDir(unitsDir)) {
    if (!ent.isFile || !ent.name.endsWith(".service")) continue;
    const dest = join(systemdDir, ent.name);
    await $`sudo systemctl disable --now ${ent.name}`.noThrow();
    await $`sudo rm -f ${dest}`.noThrow();
    console.log(`[systemd] Removed ${dest}`);
  }
  await $`sudo systemctl daemon-reload`.noThrow();
}

async function resolveUnitsForHost(
  host: string,
  units?: string[],
): Promise<string[]> {
  const repoDir = repoDirFromModules();
  const unitsDir = join(repoDir, "hosts", host, "systemd");
  if (units && units.length > 0) {
    return units.map((u) => {
      if (u.endsWith(".service")) return u;
      if (u.startsWith("psyched-")) return `${u}.service`;
      return `psyched-${u}.service`;
    });
  }
  // No units specified: enumerate generated units
  const list: string[] = [];
  try {
    for await (const ent of Deno.readDir(unitsDir)) {
      if (!ent.isFile || !ent.name.endsWith(".service")) continue;
      list.push(ent.name);
    }
  } catch {
    // no generated units
  }
  return list;
}

export async function systemdEnable(units?: string[]): Promise<void> {
  const hn = await $`hostname -s`.stdout("piped");
  const host = (hn.stdout || hn.stderr || "").toString().trim() ||
    Deno.env.get("HOST") || "default";
  // Ensure units are present by regenerating/copying
  await systemdInstall(units);
  const resolved = await resolveUnitsForHost(host, units);
  await $`sudo systemctl daemon-reload`.noThrow();
  for (const u of resolved) {
    console.log(`[systemd] Enabling ${u}`);
    await $`sudo systemctl enable --now ${u}`.noThrow();
  }
}

export async function systemdDisable(units?: string[]): Promise<void> {
  const hn = await $`hostname -s`.stdout("piped");
  const host = (hn.stdout || hn.stderr || "").toString().trim() ||
    Deno.env.get("HOST") || "default";
  const resolved = await resolveUnitsForHost(host, units);
  for (const u of resolved) {
    console.log(`[systemd] Disabling ${u}`);
    await $`sudo systemctl disable --now ${u}`.noThrow();
  }
  await $`sudo systemctl daemon-reload`.noThrow();
}

export async function systemdStart(units?: string[]): Promise<void> {
  const hn = await $`hostname -s`.stdout("piped");
  const host = (hn.stdout || hn.stderr || "").toString().trim() ||
    Deno.env.get("HOST") || "default";
  const resolved = await resolveUnitsForHost(host, units);
  for (const u of resolved) {
    console.log(`[systemd] Starting ${u}`);
    await $`sudo systemctl start ${u}`.noThrow();
  }
}

export async function systemdStop(units?: string[]): Promise<void> {
  const hn = await $`hostname -s`.stdout("piped");
  const host = (hn.stdout || hn.stderr || "").toString().trim() ||
    Deno.env.get("HOST") || "default";
  const resolved = await resolveUnitsForHost(host, units);
  for (const u of resolved) {
    console.log(`[systemd] Stopping ${u}`);
    await $`sudo systemctl stop ${u}`.noThrow();
  }
}

export async function systemdReload(units?: string[]): Promise<void> {
  const hn = await $`hostname -s`.stdout("piped");
  const host = (hn.stdout || hn.stderr || "").toString().trim() ||
    Deno.env.get("HOST") || "default";
  const resolved = await resolveUnitsForHost(host, units);
  await $`sudo systemctl daemon-reload`.noThrow();
  for (const u of resolved) {
    console.log(`[systemd] Reloading ${u}`);
    await $`sudo systemctl reload ${u}`.noThrow();
  }
}

export async function systemdRestart(units?: string[]): Promise<void> {
  const hn = await $`hostname -s`.stdout("piped");
  const host = (hn.stdout || hn.stderr || "").toString().trim() ||
    Deno.env.get("HOST") || "default";
  const resolved = await resolveUnitsForHost(host, units);
  for (const u of resolved) {
    console.log(`[systemd] Restarting ${u}`);
    await $`sudo systemctl restart ${u}`.noThrow();
  }
}

export async function systemdDebug(units?: string[]): Promise<void> {
  const hn = await $`hostname -s`.stdout("piped");
  const host = (hn.stdout || hn.stderr || "").toString().trim() ||
    Deno.env.get("HOST") || "default";
  const resolved = await resolveUnitsForHost(host, units);
  if (resolved.length === 0) {
    console.log("[systemd] No units to debug");
    return;
  }
  for (const u of resolved) {
    console.log(`\n[systemd] Status for ${u}:`);
    await $`sudo systemctl status ${u}`.noThrow();
    console.log(`\n[systemd] Journal for ${u}:`);
    await $`sudo journalctl -u ${u} -n 200 -e --no-pager`.noThrow();
  }
}
