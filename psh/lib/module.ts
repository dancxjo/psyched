import { dirname, join, relative, resolve } from "$std/path/mod.ts";
import { parse as parseToml } from "$std/toml/mod.ts";
import { walkSync } from "$std/fs/walk.ts";
import { colors } from "$cliffy/ansi/colors.ts";
import { $ } from "$dax";
import { modulesRoot, repoRoot, workspaceRoot, workspaceSrc } from "./paths.ts";

const PID_DIR = join(workspaceRoot(), ".psh");
const PILOT_MANIFEST = "fresh.gen.ts";

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
  pilot_control?: string;
  git?: GitRepo[];
  ros?: RosBuildConfig;
}

export interface ModuleManifest {
  unit: Record<string, UnitConfig>;
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

async function installAptPackages(
  label: string,
  packages: string[] = [],
): Promise<void> {
  if (!packages.length) return;
  const expanded = packages.map((pkg) => expandEnvPlaceholders(pkg).trim())
    .filter(Boolean);
  if (!expanded.length) return;
  console.log(
    colors.cyan(`[${label}] Installing apt packages: ${expanded.join(", ")}`),
  );
  const useSudo = await $`command -v sudo`.quiet().noThrow().then((res) =>
    res.code === 0
  );
  const cmd = useSudo
    ? $`sudo apt-get install -y ${expanded}`
    : $`apt-get install -y ${expanded}`;
  await cmd.env({ DEBIAN_FRONTEND: "noninteractive" }).stdout("inherit").stderr(
    "inherit",
  );
}

async function installPipPackages(
  label: string,
  packages: string[] = [],
): Promise<void> {
  if (!packages.length) return;
  const expanded = packages.map((pkg) => expandEnvPlaceholders(pkg).trim())
    .filter(Boolean);
  if (!expanded.length) return;
  console.log(
    colors.cyan(`[${label}] Installing pip packages: ${expanded.join(", ")}`),
  );
  const pip = await $`command -v pip3`.quiet().noThrow();
  const pipCmd = pip.code === 0 ? "pip3" : "pip";
  await $`${pipCmd} install ${expanded}`.stdout("inherit").stderr("inherit");
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
      await $`command -v ${expanded}`.quiet().noThrow().then((r) =>
        r.code === 0
      )
    ) {
      return expanded;
    }
    if (pathExists(expanded)) {
      return expanded;
    }
  }
  return resolve(join(moduleDir, script));
}

function locatePilotFrontend(): string {
  const target = join(repoRoot(), "modules", "pilot", "frontend");
  if (!pathExists(target)) {
    throw new Error("pilot frontend not found; run within repository");
  }
  return target;
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
      throw new Error(`refusing to replace non-symlink at ${destination}`);
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

async function linkPilotAssets(
  module: string,
  moduleDir: string,
): Promise<void> {
  const pilotDir = join(moduleDir, "pilot");
  if (!pathExists(pilotDir)) {
    console.log(
      colors.yellow(`[${module}] no pilot overlay directory; skipping.`),
    );
    return;
  }
  const overlayRoot = locatePilotFrontend();
  console.log(colors.cyan(`[${module}] linking pilot assets from ${pilotDir}`));
  for (const entry of walkSync(pilotDir)) {
    if (entry.path === pilotDir) continue;
    if (entry.isDirectory) continue;
    const relativePath = relative(pilotDir, entry.path);
    const destination = join(overlayRoot, relativePath);
    ensureDirectory(dirname(destination));
    link(entry.path, destination);
  }
  await rebuildPilotManifest(overlayRoot);
}

async function rebuildPilotManifest(frontendRoot: string): Promise<void> {
  const manifestPath = join(frontendRoot, PILOT_MANIFEST);
  const imports: string[] = [];
  const routesDir = join(frontendRoot, "routes");
  for (const entry of walkSync(routesDir, { includeDirs: false })) {
    if (!entry.path.endsWith(".tsx") && !entry.path.endsWith(".ts")) continue;
    const rel = entry.path.replace(`${frontendRoot}/`, "");
    imports.push(rel);
  }
  imports.sort();
  const buffer = `// Autogenerated by psh\nexport const manifest = ${
    JSON.stringify(imports, null, 2)
  } as const;\n`;
  await Deno.writeTextFile(manifestPath, buffer);
}

export async function setupModule(module: string): Promise<void> {
  const moduleDir = locateModuleDir(module);
  const config = loadModuleManifest(moduleDir, module);
  await installAptPackages(`setup:${module}`, dedupePreserveOrder(config.apt));
  await installPipPackages(`setup:${module}`, dedupePreserveOrder(config.pip));
  await syncGitRepos(module, config.git);
  linkModulePackages(module, moduleDir);
  if (config.patches) {
    for (const patch of config.patches) {
      await runScript(module, patch, moduleDir);
    }
  }
  await prepareRosWorkspace(module, config.ros);
  await linkPilotAssets(module, moduleDir);
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

async function prepareRosWorkspace(
  module: string,
  ros?: RosBuildConfig,
): Promise<void> {
  if (!ros) return;
  const base = ros.workspace
    ? (ros.workspace.startsWith("/")
      ? ros.workspace
      : resolve(join(workspaceRoot(), ros.workspace)))
    : workspaceRoot();
  if (!pathExists(base)) {
    throw new Error(`ROS workspace ${base} not found for module '${module}'`);
  }
  const srcDir = join(base, "src");
  ensureDirectory(srcDir);
  if (!ros.skip_rosdep) {
    console.log(colors.cyan(`[${module}] running rosdep install in ${base}`));
    const args = [
      "install",
      "--from-paths",
      srcDir,
      "--ignore-src",
      "-r",
      "-y",
    ];
    if (ros.skip_rosdep_keys && ros.skip_rosdep_keys.length) {
      args.push("--skip-keys", ros.skip_rosdep_keys.join(" "));
    }
    await $`rosdep ${args}`.cwd(base).stdout("inherit").stderr("inherit");
  } else {
    console.log(
      colors.yellow(`[${module}] skipping rosdep (skip_rosdep=true)`),
    );
  }
  if (ros.skip_colcon) {
    console.log(
      colors.yellow(`[${module}] skipping colcon build (skip_colcon=true)`),
    );
    return;
  }
  const colconArgs = ["build", "--symlink-install"];
  if (ros.build_args?.length) {
    colconArgs.push(...ros.build_args);
  }
  if (ros.packages?.length) {
    colconArgs.push("--packages-select", ...ros.packages);
  }
  await $`colcon ${colconArgs}`.cwd(base).stdout("inherit").stderr("inherit");
}

export async function teardownModule(module: string): Promise<void> {
  const moduleDir = locateModuleDir(module);
  await bringModuleDown(module);
  unlinkModulePackages(module, moduleDir);
  await unlinkPilotAssets(module, moduleDir);
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

async function unlinkPilotAssets(
  module: string,
  moduleDir: string,
): Promise<void> {
  const pilotDir = join(moduleDir, "pilot");
  if (!pathExists(pilotDir)) return;
  const overlayRoot = locatePilotFrontend();
  console.log(colors.yellow(`[${module}] removing pilot assets`));
  for (const entry of walkSync(pilotDir, { includeDirs: false })) {
    const relativePath = relative(pilotDir, entry.path);
    const destination = join(overlayRoot, relativePath);
    unlink(destination);
  }
  await rebuildPilotManifest(overlayRoot);
}

export async function bringModuleUp(module: string): Promise<void> {
  const moduleDir = locateModuleDir(module);
  const config = loadModuleManifest(moduleDir, module);
  if (!config.launch) {
    throw new Error(`module '${module}' has no launch script configured`);
  }
  const launchScript = await resolveModuleScript(config.launch, moduleDir);
  console.log(colors.green(`==> Launching module '${module}'`));
  const child = new Deno.Command("bash", {
    args: [launchScript],
    cwd: moduleDir,
    stdout: "inherit",
    stderr: "inherit",
  }).spawn();
  writePid(module, child.pid);
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
    if (entry.isDirectory) {
      names.push(entry.name);
    }
  }
  names.sort();
  return names;
}

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
  for (const module of modules) {
    await setupModule(module);
  }
}

export async function teardownModules(modules: string[]): Promise<void> {
  for (const module of modules) {
    await teardownModule(module);
  }
}

export async function bringModulesUp(modules: string[]): Promise<void> {
  for (const module of modules) {
    await bringModuleUp(module);
  }
}

export async function bringModulesDown(modules: string[]): Promise<void> {
  for (const module of modules) {
    await bringModuleDown(module);
  }
}
