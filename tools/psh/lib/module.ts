import { delay } from "$std/async/delay.ts";
import { dirname, join, relative, resolve } from "$std/path/mod.ts";
import { parse as parseToml } from "$std/toml/mod.ts";
import { walkSync } from "$std/fs/walk.ts";
import { colors } from "$cliffy/ansi/colors.ts";
import { $ } from "$dax";
import { modulesRoot, repoRoot, workspaceRoot, workspaceSrc } from "./paths.ts";
import { ensureRebootCompleted } from "./reboot_guard.ts";

const PID_DIR = join(workspaceRoot(), ".psh");

type ModuleDirective = {
  name?: string;
  env?: Record<string, unknown>;
};

type HostModuleEnv = Record<string, string>;

const MODULE_ENV_CACHE = new Map<string, HostModuleEnv>();

function sanitizeEnvRecord(
  input?: Record<string, unknown>,
): Record<string, string> {
  if (!input) return {};
  const result: Record<string, string> = {};
  for (const [key, value] of Object.entries(input)) {
    if (!key) continue;
    if (value === undefined || value === null) continue;
    result[key] = typeof value === "string" ? value : String(value);
  }
  return result;
}

function hostCandidates(): string[] {
  const seen = new Set<string>();
  const ordered: string[] = [];
  const override = Deno.env.get("PSH_HOST")?.trim();
  if (override && !seen.has(override)) {
    seen.add(override);
    ordered.push(override);
  }
  try {
    const hostname = Deno.hostname()?.trim();
    if (hostname && !seen.has(hostname)) {
      seen.add(hostname);
      ordered.push(hostname);
    }
  } catch (_error) {
    // Hostname detection can fail in restricted environments; ignore.
  }
  return ordered;
}

function envVerbose(): boolean {
  return Deno.env.get("PSH_VERBOSE") === "1";
}

async function loadHostModuleDirective(
  module: string,
): Promise<ModuleDirective | undefined> {
  const { HostConfigNotFoundError, loadHostConfig } = await import("./host.ts");
  for (const candidate of hostCandidates()) {
    try {
      const { config } = loadHostConfig(candidate);
      const directive = config.modules?.find((entry) => entry.name === module);
      if (directive) {
        return directive as ModuleDirective;
      }
    } catch (error) {
      if (error instanceof HostConfigNotFoundError) {
        continue;
      }
      if (envVerbose()) {
        const message = error instanceof Error ? error.message : String(error);
        console.warn(
          `[${module}] host config lookup failed for '${candidate}': ${message}`,
        );
      }
    }
  }
  return undefined;
}

async function moduleEnvOverrides(module: string): Promise<HostModuleEnv> {
  if (MODULE_ENV_CACHE.has(module)) {
    return MODULE_ENV_CACHE.get(module)!;
  }
  try {
    const directive = await loadHostModuleDirective(module);
    const env = sanitizeEnvRecord(directive?.env);
    MODULE_ENV_CACHE.set(module, env);
    return env;
  } catch (error) {
    if (envVerbose()) {
      const message = error instanceof Error ? error.message : String(error);
      console.warn(`[${module}] failed to resolve host environment: ${message}`);
    }
    MODULE_ENV_CACHE.set(module, {});
    return {};
  }
}

function resetModuleEnvCache(): void {
  MODULE_ENV_CACHE.clear();
}

export interface GitRepo {
  url: string;
  branch?: string;
  path?: string;
}

export interface RosBuildConfig {
  workspace?: string;
  packages?: string[];
  build_args?: string[];
  skip_rosdep_keys?: string[];
  skip_rosdep?: boolean;
  skip_colcon?: boolean;
}

export interface UnitConfig {
  apt?: string[];
  pip?: string[];
  patches?: string[];
  launch?: string;
  shutdown?: string;
  cockpit_control?: string;
  git?: GitRepo[];
  ros?: RosBuildConfig;
}

export interface ModuleManifest {
  unit: Record<string, UnitConfig>;
}

export interface BringModuleUpOptions {
  verbose?: boolean;
}

export interface ModuleLaunchFailure {
  module: string;
  error: unknown;
}

export interface BringModulesUpOptions extends BringModuleUpOptions {
  launcher?: (
    module: string,
    options: BringModuleUpOptions,
  ) => Promise<void>;
  onError?: (failure: ModuleLaunchFailure) => void;
}

function ensurePidDir(): void {
  Deno.mkdirSync(PID_DIR, { recursive: true });
}

function pidPath(module: string): string {
  ensurePidDir();
  return join(PID_DIR, `${module}.pid`);
}

function writePid(module: string, pid: number): void {
  Deno.writeTextFileSync(pidPath(module), `${pid}`);
}

function readPid(module: string): number | null {
  try {
    const content = Deno.readTextFileSync(pidPath(module)).trim();
    if (!content) {
      return null;
    }
    const parsed = Number(content);
    return Number.isNaN(parsed) ? null : parsed;
  } catch (error) {
    if (error instanceof Deno.errors.NotFound) {
      return null;
    }
    throw error;
  }
}

function clearPid(module: string): void {
  try {
    Deno.removeSync(pidPath(module));
  } catch (error) {
    if (!(error instanceof Deno.errors.NotFound)) {
      throw error;
    }
  }
}

function pathExists(path: string): boolean {
  try {
    Deno.statSync(path);
    return true;
  } catch (error) {
    if (error instanceof Deno.errors.NotFound) {
      return false;
    }
    throw error;
  }
}

function ensureDirectory(path: string): void {
  Deno.mkdirSync(path, { recursive: true });
}

function fileContentsEqual(pathA: string, pathB: string): boolean {
  try {
    const a = Deno.readFileSync(pathA);
    const b = Deno.readFileSync(pathB);
    if (a.length !== b.length) {
      return false;
    }
    for (let i = 0; i < a.length; i++) {
      if (a[i] !== b[i]) {
        return false;
      }
    }
    return true;
  } catch (error) {
    if (error instanceof Deno.errors.NotFound) {
      return false;
    }
    throw error;
  }
}

function ensureLogFile(path: string): void {
  ensureDirectory(dirname(path));
  const file = Deno.openSync(path, {
    create: true,
    write: true,
    append: true,
  });
  file.close();
}

function shellEscape(value: string): string {
  return `'${value.replace(/'/g, `'"'"'`)}'`;
}

export interface ComposeLaunchCommandOptions {
  envCommands: string[];
  launchScript: string;
  logFile: string;
  module: string;
}

const EARLY_EXIT_WINDOW_MS = 500;

export async function awaitModuleStability(
  statusPromise: Promise<Deno.CommandStatus>,
  windowMs = EARLY_EXIT_WINDOW_MS,
): Promise<Deno.CommandStatus | null> {
  const outcome = await Promise.race<
    { status: Deno.CommandStatus } | null
  >([
    statusPromise.then((status) => ({ status })),
    delay(windowMs).then(() => null),
  ]);

  return outcome?.status ?? null;
}

export interface LaunchDiagnosticsContext {
  module: string;
  launchScript: string;
  envCommands: string[];
  logFile: string;
  command: string;
  envOverrides?: Record<string, string>;
}

export function formatLaunchDiagnostics(
  context: LaunchDiagnosticsContext,
): string[] {
  const lines = [
    `module: ${context.module}`,
    `launch script: ${context.launchScript}`,
    `log file: ${context.logFile}`,
  ];
  if (context.envCommands.length) {
    lines.push(`environment bootstrap: ${context.envCommands.join(" && ")}`);
  } else {
    lines.push("environment bootstrap: (none)");
  }
  const overrides = context.envOverrides ?? {};
  const entries = Object.entries(overrides);
  if (entries.length) {
    entries.sort(([a], [b]) => a.localeCompare(b));
    lines.push(
      `module env overrides: ${entries.map(([k, v]) => `${k}=${v}`).join(", ")}`,
    );
  } else {
    lines.push("module env overrides: (none)");
  }
  lines.push(`spawn command: ${context.command}`);
  return lines;
}

export function formatExitSummary(
  module: string,
  status: Deno.CommandStatus,
): string {
  const summary = status.success
    ? `[${module}] exited cleanly (code ${status.code ?? 0})`
    : `[${module}] exited with code ${status.code ?? "unknown"}` +
    (status.signal ? ` (signal ${status.signal})` : "");
  return status.success ? colors.yellow(summary) : colors.red(summary);
}

function logExitSummary(module: string, status: Deno.CommandStatus): void {
  console.log(formatExitSummary(module, status));
}

/**
 * Build the shell pipeline used to bootstrap a module launch.
 *
 * The returned command performs environment setup and hands control to the
 * module's launch script so it can manage its own lifecycle and logging.
 */
export function composeLaunchCommand(
  options: ComposeLaunchCommandOptions,
): string {
  const prefixLogs = join(
    repoRoot(),
    "tools",
    "psh",
    "scripts",
    "prefix_logs.sh",
  );
  const moduleName = shellEscape(options.module);
  const logFile = shellEscape(options.logFile);
  const prefixScript = shellEscape(prefixLogs);
  const stdoutPipe = `${prefixScript} ${moduleName} stdout | tee -a ${logFile}`;
  const stderrPipe = `${prefixScript} ${moduleName} stderr | tee -a ${logFile}`;
  const parts = [
    ...options.envCommands,
    `exec ${shellEscape(options.launchScript)
    } > >(${stdoutPipe}) 2> >(${stderrPipe})`,
  ];
  return parts.join(" && ");
}

export function locateModuleDir(module: string): string {
  const candidate = join(modulesRoot(), module);
  if (!pathExists(candidate)) {
    throw new Error(`module '${module}' not found under modules/`);
  }
  return candidate;
}

function loadModuleManifest(moduleDir: string, module: string): UnitConfig {
  const manifestPath = join(moduleDir, "module.toml");
  if (!pathExists(manifestPath)) {
    throw new Error(
      `module '${module}' missing module.toml at ${manifestPath}`,
    );
  }
  const manifestContent = Deno.readTextFileSync(manifestPath);
  const parsed = parseToml(manifestContent) as unknown as ModuleManifest;
  if (!parsed.unit || typeof parsed.unit !== "object") {
    throw new Error(
      `module '${module}' has malformed module.toml (unit table missing)`,
    );
  }
  const config = parsed.unit[module];
  if (!config) {
    throw new Error(
      `module '${module}' missing unit.${module} section in module.toml`,
    );
  }
  return config;
}

export function moduleConfig(module: string): UnitConfig {
  const dir = locateModuleDir(module);
  return loadModuleManifest(dir, module);
}

function dedupePreserveOrder(items: string[] = []): string[] {
  const seen = new Set<string>();
  const result: string[] = [];
  for (const item of items) {
    if (item && !seen.has(item)) {
      seen.add(item);
      result.push(item);
    }
  }
  return result;
}

function expandEnvPlaceholders(input: string): string {
  return input.replace(/\$\{([A-Za-z0-9_]+)\}/g, (_match, name: string) => {
    const value = Deno.env.get(name);
    if (value === undefined) {
      throw new Error(`environment variable ${name} is not set`);
    }
    return value;
  });
}

function formatSetupLabel(modules: string[]): string {
  if (!modules.length) return "setup";
  if (modules.length === 1) return `setup:${modules[0]}`;
  return `setup:${modules.join(", ")}`;
}

interface AptInstallInvocation {
  modules: string[];
  packages: string[];
}

type AptInstallRunner = (invocation: AptInstallInvocation) => Promise<void>;

const defaultAptInstallRunner: AptInstallRunner = async (
  invocation: AptInstallInvocation,
): Promise<void> => {
  const label = formatSetupLabel(invocation.modules);
  console.log(
    colors.cyan(
      `[${label}] Installing apt packages: ${invocation.packages.join(", ")}`,
    ),
  );
  const useSudo = await $`which sudo`.noThrow().stdout("null").then((
    res: { code: number },
  ) => res.code === 0);
  const cmd = useSudo
    ? $`sudo apt-get install -y ${invocation.packages}`
    : $`apt-get install -y ${invocation.packages}`;
  await cmd.env({ DEBIAN_FRONTEND: "noninteractive" }).stdout("inherit").stderr(
    "inherit",
  );
};

export class AptPackagePlanner {
  #modules: string[] = [];
  #packages: string[] = [];
  #runner: AptInstallRunner;

  constructor(runner: AptInstallRunner = defaultAptInstallRunner) {
    this.#runner = runner;
  }

  add(module: string, packages: string[]): void {
    const expanded = packages.map((pkg) => expandEnvPlaceholders(pkg).trim())
      .filter(Boolean);
    if (!expanded.length) return;
    if (!this.#modules.includes(module)) {
      this.#modules.push(module);
    }
    for (const pkg of expanded) {
      if (!this.#packages.includes(pkg)) {
        this.#packages.push(pkg);
      }
    }
  }

  async execute(): Promise<void> {
    if (!this.#packages.length) return;
    await this.#runner({
      modules: [...this.#modules],
      packages: [...this.#packages],
    });
    this.#modules = [];
    this.#packages = [];
  }
}

async function installAptPackages(
  module: string,
  packages: string[] = [],
  planner?: AptPackagePlanner,
): Promise<void> {
  if (!packages.length) return;
  if (planner) {
    planner.add(module, packages);
    return;
  }
  const immediate = new AptPackagePlanner();
  immediate.add(module, packages);
  await immediate.execute();
}

interface PipInstallInvocation {
  modules: string[];
  packages: string[];
}

type PipInstallRunner = (invocation: PipInstallInvocation) => Promise<void>;

const defaultPipInstallRunner: PipInstallRunner = async (
  invocation: PipInstallInvocation,
): Promise<void> => {
  const label = formatSetupLabel(invocation.modules);
  console.log(
    colors.cyan(
      `[${label}] Installing pip packages: ${invocation.packages.join(", ")}`,
    ),
  );
  const rosDistro = (Deno.env.get("ROS_DISTRO") ?? "kilted").trim();
  const colconPip = `/opt/ros/${rosDistro}/colcon-venv/bin/pip`;
  if (pathExists(colconPip)) {
    const useSudo = await $`which sudo`.noThrow().stdout("null").then((
      res: { code: number },
    ) => res.code === 0);
    const installer = useSudo
      ? $`sudo ${colconPip} install --no-cache-dir --upgrade ${invocation.packages}`
      : $`${colconPip} install --no-cache-dir --upgrade ${invocation.packages}`;
    await installer.stdout("inherit").stderr("inherit");
    return;
  }
  const pip = await $`which pip3`.noThrow().stdout("null");
  const pipCmd = pip.code === 0 ? "pip3" : "pip";
  await $`${pipCmd} install --break-system-packages ${invocation.packages}`
    .stdout(
      "inherit",
    ).stderr("inherit");
};

export class PipPackagePlanner {
  #modules: string[] = [];
  #packages: string[] = [];
  #runner: PipInstallRunner;

  constructor(runner: PipInstallRunner = defaultPipInstallRunner) {
    this.#runner = runner;
  }

  add(module: string, packages: string[]): void {
    const expanded = packages.map((pkg) => expandEnvPlaceholders(pkg).trim())
      .filter(Boolean);
    if (!expanded.length) return;
    if (!this.#modules.includes(module)) {
      this.#modules.push(module);
    }
    for (const pkg of expanded) {
      if (!this.#packages.includes(pkg)) {
        this.#packages.push(pkg);
      }
    }
  }

  async execute(): Promise<void> {
    if (!this.#packages.length) return;
    await this.#runner({
      modules: [...this.#modules],
      packages: [...this.#packages],
    });
    this.#modules = [];
    this.#packages = [];
  }
}

async function installPipPackages(
  module: string,
  packages: string[] = [],
  planner?: PipPackagePlanner,
): Promise<void> {
  if (!packages.length) return;
  if (planner) {
    planner.add(module, packages);
    return;
  }
  const immediate = new PipPackagePlanner();
  immediate.add(module, packages);
  await immediate.execute();
}

function deriveRepoDirname(url: string): string {
  const withoutQuery = url.split("?")[0];
  const withoutSuffix = withoutQuery.replace(/\.git$/, "");
  const segments = withoutSuffix.split("/");
  return segments[segments.length - 1];
}

async function syncGitRepos(
  module: string,
  repos: GitRepo[] = [],
): Promise<void> {
  if (!repos.length) return;
  const srcDir = workspaceSrc();
  ensureDirectory(srcDir);
  for (const repo of repos) {
    const destination = repo.path
      ? (repo.path.startsWith("/")
        ? resolve(repo.path)
        : resolve(join(srcDir, repo.path)))
      : resolve(join(srcDir, deriveRepoDirname(repo.url)));
    ensureDirectory(dirname(destination));
    const branchDesc = repo.branch ?? "<default>";
    console.log(
      colors.cyan(
        `[${module}] Syncing git repo ${repo.url} (branch ${branchDesc}) into ${destination}`,
      ),
    );
    if (pathExists(destination) && pathExists(join(destination, ".git"))) {
      const fetchArgs = repo.branch
        ? ["fetch", "origin", repo.branch]
        : ["fetch", "--all"];
      await $`git -C ${destination} ${fetchArgs}`.stdout("inherit").stderr(
        "inherit",
      );
      const pullArgs = repo.branch
        ? ["reset", "--hard", `origin/${repo.branch}`]
        : ["pull", "--ff-only"];
      await $`git -C ${destination} ${pullArgs}`.stdout("inherit").stderr(
        "inherit",
      );
    } else {
      const cloneArgs = repo.branch
        ? [
          "clone",
          "--branch",
          repo.branch,
          "--single-branch",
          repo.url,
          destination,
        ]
        : ["clone", repo.url, destination];
      await $`git ${cloneArgs}`.stdout("inherit").stderr("inherit");
    }
  }
}

async function runScript(
  module: string,
  script: string,
  moduleDir: string,
): Promise<void> {
  const resolved = await resolveModuleScript(script, moduleDir);
  if (!pathExists(resolved)) {
    throw new Error(`[${module}] script not found: ${resolved}`);
  }
  console.log(colors.cyan(`[${module}] running ${resolved}`));
  await $`bash ${resolved}`.cwd(moduleDir).stdout("inherit").stderr("inherit");
}

export async function resolveModuleScript(
  script: string,
  moduleDir: string,
): Promise<string> {
  const search = [script, join(moduleDir, script), join(repoRoot(), script)];
  for (const candidate of search) {
    const expanded = candidate.startsWith("/") ? candidate : resolve(candidate);
    if (
      await $`which ${expanded}`.noThrow().stdout("null").then((
        r: { code: number },
      ) => r.code === 0)
    ) {
      return expanded;
    }
    if (pathExists(expanded)) {
      return expanded;
    }
  }
  return resolve(join(moduleDir, script));
}

function locateCockpitFrontend(): string {
  const primary = join(repoRoot(), "modules", "cockpit", "frontend");
  if (pathExists(primary)) {
    return primary;
  }
  const fallback = join(
    repoRoot(),
    "modules",
    "cockpit",
    "packages",
    "cockpit",
    "cockpit",
    "frontend",
  );
  if (pathExists(fallback)) {
    return fallback;
  }
  throw new Error("cockpit frontend not found; run within repository");
}

function link(target: string, destination: string): void {
  try {
    const info = Deno.lstatSync(destination);
    if (info.isSymlink) {
      const linkTarget = Deno.readLinkSync(destination);
      if (linkTarget === target) {
        return;
      }
      Deno.removeSync(destination);
    } else {
      if (fileContentsEqual(target, destination)) {
        return;
      }
      console.warn(
        colors.yellow(
          `skipping link for ${destination}; destination is not a symlink`,
        ),
      );
      return;
    }
  } catch (error) {
    if (!(error instanceof Deno.errors.NotFound)) {
      throw error;
    }
  }
  ensureDirectory(dirname(destination));
  Deno.symlinkSync(target, destination);
}

function unlink(path: string): void {
  try {
    const info = Deno.lstatSync(path);
    if (info.isSymlink) {
      Deno.removeSync(path);
    }
  } catch (error) {
    if (!(error instanceof Deno.errors.NotFound)) {
      throw error;
    }
  }
}

function linkCockpitAssets(
  module: string,
  moduleDir: string,
): void {
  const cockpitDir = join(moduleDir, "cockpit");
  if (!pathExists(cockpitDir)) {
    console.log(
      colors.yellow(`[${module}] no cockpit overlay directory; skipping.`),
    );
    return;
  }
  const overlayRoot = locateCockpitFrontend();
  console.log(
    colors.cyan(`[${module}] linking cockpit assets from ${cockpitDir}`),
  );
  for (const entry of walkSync(cockpitDir)) {
    if (entry.path === cockpitDir) continue;
    if (entry.isDirectory) continue;
    const relativePath = relative(cockpitDir, entry.path);
    const destination = join(overlayRoot, relativePath);
    ensureDirectory(dirname(destination));
    link(entry.path, destination);
  }
}

export interface ModuleSetupOptions {
  aptPlanner?: AptPackagePlanner;
  pipPlanner?: PipPackagePlanner;
  rosPlanner?: RosBuildPlanner;
}

export async function setupModule(
  module: string,
  options: ModuleSetupOptions = {},
): Promise<void> {
  const moduleDir = locateModuleDir(module);
  const config = loadModuleManifest(moduleDir, module);
  await installAptPackages(
    module,
    dedupePreserveOrder(config.apt),
    options.aptPlanner,
  );
  await installPipPackages(
    module,
    dedupePreserveOrder(config.pip),
    options.pipPlanner,
  );
  await syncGitRepos(module, config.git);
  linkModulePackages(module, moduleDir);
  if (config.patches) {
    for (const patch of config.patches) {
      await runScript(module, patch, moduleDir);
    }
  }
  await prepareRosWorkspace(module, config.ros, options.rosPlanner);
  await linkCockpitAssets(module, moduleDir);
}

function linkModulePackages(
  module: string,
  moduleDir: string,
): void {
  const packagesDir = join(moduleDir, "packages");
  if (!pathExists(packagesDir)) {
    console.log(
      colors.yellow(`[${module}] packages directory missing; skipping link`),
    );
    return;
  }
  const srcRoot = workspaceSrc();
  ensureDirectory(srcRoot);
  let linkedAny = false;
  for (const entry of Deno.readDirSync(packagesDir)) {
    if (!entry.isDirectory) continue;
    const packagePath = join(packagesDir, entry.name);
    if (
      !pathExists(join(packagePath, "package.xml")) &&
      !pathExists(join(packagePath, "Cargo.toml"))
    ) {
      continue;
    }
    const linkPath = join(srcRoot, entry.name);
    link(packagePath, linkPath);
    linkedAny = true;
  }
  if (linkedAny) {
    console.log(
      colors.green(`[${module}] linked ROS packages into ${srcRoot}`),
    );
  }
}

interface RosWorkspacePlan {
  workspace: string;
  modules: string[];
  rosdepRequired: boolean;
  rosdepSkipKeys: string[];
  colconRequired: boolean;
  colconPackages: string[];
  colconArgs: string[];
}

export interface RosdepInvocation {
  workspace: string;
  srcDir: string;
  skipKeys: string[];
  modules: string[];
}

export interface ColconInvocation {
  workspace: string;
  packages: string[];
  buildArgs: string[];
  modules: string[];
}

export interface RosBuildPlannerRunner {
  rosdep(invocation: RosdepInvocation): Promise<void>;
  colcon(invocation: ColconInvocation): Promise<void>;
}

const defaultRosBuildRunner: RosBuildPlannerRunner = {
  async rosdep({ workspace, srcDir, skipKeys }: RosdepInvocation) {
    const args = [
      "install",
      "--from-paths",
      srcDir,
      "--ignore-src",
      "-r",
      "-y",
    ];
    if (skipKeys.length) {
      args.push("--skip-keys", skipKeys.join(" "));
    }
    await $`rosdep ${args}`.cwd(workspace).stdout("inherit").stderr("inherit");
  },
  async colcon({ workspace, packages, buildArgs }: ColconInvocation) {
    const args = ["build", "--symlink-install"];
    if (packages.length) {
      args.push("--packages-select", ...packages);
    }
    if (buildArgs.length) {
      args.push(...buildArgs);
    }
    await $`colcon ${args}`.cwd(workspace).stdout("inherit").stderr("inherit");
  },
};

export class RosBuildPlanner {
  #plans = new Map<string, RosWorkspacePlan>();
  #runner: RosBuildPlannerRunner;

  constructor(runner: RosBuildPlannerRunner = defaultRosBuildRunner) {
    this.#runner = runner;
  }

  add(module: string, ros: RosBuildConfig): void {
    const workspace = this.#resolveWorkspaceBase(ros);
    if (!pathExists(workspace)) {
      throw new Error(
        `ROS workspace ${workspace} not found for module '${module}'`,
      );
    }
    const plan = this.#plans.get(workspace) ?? {
      workspace,
      modules: [],
      rosdepRequired: false,
      rosdepSkipKeys: [],
      colconRequired: false,
      colconPackages: [],
      colconArgs: [],
    };
    plan.modules.push(module);
    if (!ros.skip_rosdep) {
      plan.rosdepRequired = true;
      if (ros.skip_rosdep_keys?.length) {
        plan.rosdepSkipKeys.push(...ros.skip_rosdep_keys);
      }
    }
    if (!ros.skip_colcon) {
      plan.colconRequired = true;
      if (ros.packages?.length) {
        plan.colconPackages.push(...ros.packages);
      }
      if (ros.build_args?.length) {
        plan.colconArgs.push(...ros.build_args);
      }
    }
    this.#plans.set(workspace, plan);
  }

  async execute(): Promise<void> {
    for (const plan of this.#plans.values()) {
      const label = plan.modules.join(", ");
      const srcDir = join(plan.workspace, "src");
      ensureDirectory(srcDir);
      if (plan.rosdepRequired) {
        console.log(
          colors.cyan(`[${label}] running rosdep install in ${plan.workspace}`),
        );
        await this.#runner.rosdep({
          workspace: plan.workspace,
          srcDir,
          skipKeys: dedupePreserveOrder(plan.rosdepSkipKeys),
          modules: [...plan.modules],
        });
      } else if (plan.modules.length) {
        console.log(
          colors.yellow(`[${label}] skipping rosdep (skip_rosdep=true)`),
        );
      }
      if (plan.colconRequired) {
        console.log(
          colors.cyan(`[${label}] running colcon build in ${plan.workspace}`),
        );
        await this.#runner.colcon({
          workspace: plan.workspace,
          packages: dedupePreserveOrder(plan.colconPackages),
          buildArgs: [...plan.colconArgs],
          modules: [...plan.modules],
        });
      } else if (plan.modules.length) {
        console.log(
          colors.yellow(`[${label}] skipping colcon build (skip_colcon=true)`),
        );
      }
    }
  }

  #resolveWorkspaceBase(ros: RosBuildConfig): string {
    if (!ros.workspace) {
      return workspaceRoot();
    }
    return ros.workspace.startsWith("/")
      ? ros.workspace
      : resolve(join(workspaceRoot(), ros.workspace));
  }
}

async function prepareRosWorkspace(
  module: string,
  ros: RosBuildConfig | undefined,
  planner?: RosBuildPlanner,
): Promise<void> {
  if (!ros) return;
  if (planner) {
    planner.add(module, ros);
    return;
  }
  const singlePlanner = new RosBuildPlanner();
  singlePlanner.add(module, ros);
  await singlePlanner.execute();
}

export async function teardownModule(module: string): Promise<void> {
  const moduleDir = locateModuleDir(module);
  await bringModuleDown(module);
  unlinkModulePackages(module, moduleDir);
  await unlinkCockpitAssets(module, moduleDir);
  clearPid(module);
}

function unlinkModulePackages(
  module: string,
  moduleDir: string,
): void {
  const packagesDir = join(moduleDir, "packages");
  if (!pathExists(packagesDir)) return;
  const srcRoot = workspaceSrc();
  for (const entry of Deno.readDirSync(packagesDir)) {
    if (!entry.isDirectory) continue;
    const packagePath = join(packagesDir, entry.name);
    const linkPath = join(srcRoot, entry.name);
    try {
      const meta = Deno.lstatSync(linkPath);
      if (meta.isSymlink && Deno.readLinkSync(linkPath) === packagePath) {
        Deno.removeSync(linkPath);
      }
    } catch (error) {
      if (!(error instanceof Deno.errors.NotFound)) throw error;
    }
  }
  console.log(
    colors.yellow(`[${module}] unlinked ROS packages from ${srcRoot}`),
  );
}

function unlinkCockpitAssets(
  module: string,
  moduleDir: string,
): void {
  const cockpitDir = join(moduleDir, "cockpit");
  if (!pathExists(cockpitDir)) return;
  const overlayRoot = locateCockpitFrontend();
  console.log(colors.yellow(`[${module}] removing cockpit assets`));
  for (const entry of walkSync(cockpitDir, { includeDirs: false })) {
    const relativePath = relative(cockpitDir, entry.path);
    const destination = join(overlayRoot, relativePath);
    unlink(destination);
  }
}

export async function bringModuleUp(
  module: string,
  options: BringModuleUpOptions = {},
): Promise<void> {
  const moduleDir = locateModuleDir(module);
  const config = loadModuleManifest(moduleDir, module);
  if (!config.launch) {
    throw new Error(`module '${module}' has no launch script configured`);
  }
  const moduleEnv = await moduleEnvOverrides(module);
  const launchScript = await resolveModuleScript(config.launch, moduleDir);
  console.log(colors.green(`==> Launching module '${module}' in background`));

  const logFile = join(repoRoot(), "log", "modules", `${module}.log`);
  ensureLogFile(logFile);
  console.log(colors.dim(`[${module}] writing logs to ${logFile}`));

  // Build complete environment setup command
  const workspaceEnv = join(repoRoot(), "env", "psyched_env.sh");

  const envCommands: string[] = [];

  // Source environment helpers
  if (pathExists(workspaceEnv)) {
    envCommands.push(`source ${workspaceEnv}`);
    envCommands.push("psyched::activate --quiet");
  } else {
    const workspaceSetup = join(workspaceRoot(), "install", "setup.bash");
    if (pathExists(workspaceSetup)) {
      envCommands.push(`source ${workspaceSetup}`);
    } else {
      const rosDistro = Deno.env.get("ROS_DISTRO");
      if (rosDistro) {
        const systemSetup = `/opt/ros/${rosDistro}/setup.bash`;
        if (pathExists(systemSetup)) {
          envCommands.push(`source ${systemSetup}`);
        }
      }
    }
  }

  // Combine environment setup with launch script execution
  const launchCommand = composeLaunchCommand({
    envCommands,
    launchScript,
    logFile,
    module,
  });

  if (options.verbose) {
    for (
      const line of formatLaunchDiagnostics({
        module,
        launchScript,
        envCommands,
        logFile,
        command: launchCommand,
        envOverrides: moduleEnv,
      })
    ) {
      console.log(colors.dim(`[${module}] ${line}`));
    }
  }

  // Launch the script in the background, letting bash handle output
  // duplication to the module log while keeping stdout/stderr attached to the
  // terminal so the process keeps running after psh exits.
  const child = new Deno.Command("bash", {
    args: ["-c", launchCommand],
    cwd: moduleDir,
    stdin: "null",
    stdout: "inherit",
    stderr: "inherit",
    env: moduleEnv,
  }).spawn();

  writePid(module, child.pid);
  console.log(
    colors.green(
      `==> Module '${module}' started with PID ${child.pid} (logs: ${logFile})`,
    ),
  );

  const statusPromise = child.status;
  child.unref();

  const earlyStatus = await awaitModuleStability(statusPromise);

  if (earlyStatus) {
    clearPid(module);
    logExitSummary(module, earlyStatus);
    return;
  }

  statusPromise
    .then((status) => {
      clearPid(module);
      logExitSummary(module, status);
    })
    .catch((error) => {
      const message = error instanceof Error ? error.message : String(error);
      console.error(
        colors.red(`[${module}] failed to read exit status: ${message}`),
      );
    });
}

export async function bringModuleDown(module: string): Promise<void> {
  const moduleDir = locateModuleDir(module);
  const config = loadModuleManifest(moduleDir, module);
  const pid = readPid(module);
  if (pid) {
    try {
      Deno.kill(pid, "SIGTERM");
    } catch (error) {
      if (!(error instanceof Deno.errors.NotFound)) {
        console.error(
          colors.red(`[${module}] failed to terminate pid ${pid}: ${error}`),
        );
      }
    }
  }
  if (config.shutdown) {
    const shutdownScript = await resolveModuleScript(
      config.shutdown,
      moduleDir,
    );
    console.log(colors.yellow(`==> Stopping module '${module}'`));
    await $`bash ${shutdownScript}`.cwd(moduleDir).stdout("inherit").stderr(
      "inherit",
    );
  }
  clearPid(module);
}

export function listModules(): string[] {
  const names: string[] = [];
  for (const entry of Deno.readDirSync(modulesRoot())) {
    if (!entry.isDirectory) continue;
    try {
      const manifestPath = join(modulesRoot(), entry.name, "module.toml");
      // Only include directories that contain a module.toml manifest.
      if (!pathExists(manifestPath)) {
        // Skip incidental directories (e.g., work-in-progress modules).
        continue;
      }
      names.push(entry.name);
    } catch (_err) {
      // If anything unexpected happens, skip this entry to keep the
      // module listing robust during clean/teardown operations.
      continue;
    }
  }
  names.sort();
  return names;
}

export interface ModuleApiAction {
  name: string;
  description: string;
  kind?: string;
  parameters?: Record<string, unknown>;
  returns?: Record<string, unknown>;
}

export function loadModuleApiActions(): Record<string, ModuleApiAction[]> {
  const result: Record<string, ModuleApiAction[]> = {};
  const root = modulesRoot();
  try {
    Deno.statSync(root);
  } catch (_error) {
    return result;
  }

  for (const entry of Deno.readDirSync(root)) {
    if (!entry.isDirectory) continue;
    const moduleName = entry.name;
    const apiPath = join(root, moduleName, "cockpit", "api", "actions.json");
    try {
      const text = Deno.readTextFileSync(apiPath);
      const payload = JSON.parse(text);
      const rawActions = Array.isArray(payload.actions)
        ? payload.actions
        : [];
      const parsed: ModuleApiAction[] = [];
      for (const raw of rawActions) {
        if (!raw || typeof raw !== "object") continue;
        const name = typeof raw.name === "string" ? raw.name.trim() : "";
        if (!name) continue;
        const description = typeof raw.description === "string"
          ? raw.description
          : "";
        const action: ModuleApiAction = { name, description };
        if (typeof raw.kind === "string") {
          action.kind = raw.kind;
        }
        if (raw.parameters && typeof raw.parameters === "object") {
          action.parameters = raw.parameters as Record<string, unknown>;
        }
        if (raw.returns && typeof raw.returns === "object") {
          action.returns = raw.returns as Record<string, unknown>;
        }
        parsed.push(action);
      }
      if (parsed.length) {
        result[moduleName] = parsed;
      }
    } catch (error) {
      if (!(error instanceof Deno.errors.NotFound)) {
        console.warn(
          `Failed to read cockpit API for module ${moduleName}:`,
          error,
        );
      }
    }
  }

  return result;
}

export const __internals__ = {
  moduleEnvOverrides,
  resetModuleEnvCache,
};

export interface ModuleStatus {
  name: string;
  status: "running" | "stopped";
  pid?: number;
}

export function moduleStatuses(): ModuleStatus[] {
  return listModules().map((name) => {
    const pid = readPid(name);
    if (pid && isPidRunning(pid)) {
      return { name, status: "running", pid };
    }
    if (pid) clearPid(name);
    return { name, status: "stopped" };
  });
}

function isPidRunning(pid: number): boolean {
  try {
    Deno.statSync(`/proc/${pid}`);
    return true;
  } catch (error) {
    if (error instanceof Deno.errors.NotFound) {
      return false;
    }
    throw error;
  }
}

export async function setupModules(modules: string[]): Promise<void> {
  ensureRebootCompleted();
  const aptPlanner = new AptPackagePlanner();
  const pipPlanner = new PipPackagePlanner();
  const rosPlanner = new RosBuildPlanner();
  for (const module of modules) {
    await setupModule(module, { aptPlanner, pipPlanner, rosPlanner });
  }
  await aptPlanner.execute();
  await pipPlanner.execute();
  await rosPlanner.execute();
}

export async function teardownModules(modules: string[]): Promise<void> {
  for (const module of modules) {
    await teardownModule(module);
  }
}

export async function bringModulesUp(
  modules: string[],
  options: BringModulesUpOptions = {},
): Promise<void> {
  const { launcher = bringModuleUp, onError, ...rest } = options;
  const launchOptions: BringModuleUpOptions = rest;
  const failures: ModuleLaunchFailure[] = [];

  for (const module of modules) {
    try {
      await launcher(module, launchOptions);
    } catch (error) {
      const message = error instanceof Error ? error.message : String(error);
      const failure: ModuleLaunchFailure = { module, error };
      failures.push(failure);
      console.error(
        colors.red(`[${module}] failed to launch: ${message}`),
      );
      onError?.(failure);
    }
  }

  if (failures.length) {
    const summary = failures.map((failure) => failure.module).join(", ");
    const label = failures.length === 1 ? "module" : "modules";
    throw new AggregateError(
      failures.map((failure) => failure.error),
      `Failed to launch ${failures.length} ${label}: ${summary}`,
    );
  }
}

export async function bringModulesDown(modules: string[]): Promise<void> {
  for (const module of modules) {
    await bringModuleDown(module);
  }
}
