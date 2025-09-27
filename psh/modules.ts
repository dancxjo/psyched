import { parse as parseToml } from "@std/toml";
import { parse as parseYaml, stringify as stringifyYaml } from "@std/yaml";
import { dirname, fromFileUrl, join } from "@std/path";
import { $, type DaxTemplateTag, runWithStreamingTee } from "./util.ts";
import { colors } from "@cliffy/ansi/colors";

let commandRunner: DaxTemplateTag = $;

/**
 * Override the command runner used for executing shell commands. Primarily
 * exposed for tests so they can capture invocations instead of running real
 * commands.
 */
export function setModuleCommandRunner(runner: DaxTemplateTag): void {
  commandRunner = runner;
}

/** Reset the command runner back to the default Dax tag. */
export function resetModuleCommandRunner(): void {
  commandRunner = $;
}

interface ModuleActionBase {
  type: string;
  description?: string;
  cwd?: string;
  optional?: boolean;
}

interface LinkPackagesAction extends ModuleActionBase {
  type: "link_packages";
  packages: string[];
  base?: string;
  destination?: string;
}

interface GitCloneAction extends ModuleActionBase {
  type: "git_clone";
  repo: string;
  dest: string;
  branch?: string;
  tag?: string;
  depth?: number;
  base?: string;
  sparse?: boolean;
}

interface AptInstallAction extends ModuleActionBase {
  type: "apt_install";
  packages: string[];
  update?: boolean;
}

interface PipInstallAction extends ModuleActionBase {
  type: "pip_install";
  packages: string[];
  import_check?: string[];
  python?: string;
  break_system?: boolean;
  user?: boolean;
}

interface RosRunAction extends ModuleActionBase {
  type: "ros_run";
  package: string;
  executable: string;
  args?: string[];
}

interface RunAction extends ModuleActionBase {
  type: "run";
  command?: string;
  script?: string;
  shell?: string;
  env?: Record<string, string>;
}

type ModuleAction =
  | LinkPackagesAction
  | GitCloneAction
  | AptInstallAction
  | PipInstallAction
  | RosRunAction
  | RunAction;

type AptInstallHandler = (
  action: AptInstallAction,
  ctx: ModuleContext,
) => Promise<void>;

type PipInstallHandler = (
  action: PipInstallAction,
  ctx: ModuleContext,
) => Promise<void>;

let aptInstallHandler: AptInstallHandler = applyAptInstall;
let pipInstallHandler: PipInstallHandler = applyPipInstall;

interface ModuleSystemdSpec {
  description?: string;
  launch?: string;
  shutdown?: string;
  launch_command?: string;
  shutdown_command?: string;
  environment?: Record<string, string>;
  after?: string[];
  wants?: string[];
  restart?: string;
  restart_sec?: number;
  user?: string;
  working_directory?: string;
  kill_mode?: string;
  timeout_stop_sec?: number;
}

interface ModuleSpec {
  name?: string;
  description?: string;
  actions?: ModuleAction[];
  systemd?: ModuleSystemdSpec;
}

interface ModuleContext {
  module: string;
  repoDir: string;
  moduleDir: string;
  srcDir: string;
  moduleConfig: Record<string, unknown> | null;
  moduleConfigPath: string | null;
  moduleConfigOwned: boolean;
}

function resolvePath(base: string, rel?: string | null): string {
  if (!rel) return base;
  if (rel.startsWith("/")) return rel;
  return join(base, rel);
}

async function runCommand(command: string, options: {
  description?: string;
  cwd?: string;
  env?: Record<string, string>;
  optional?: boolean;
} = {}): Promise<void> {
  const { description, cwd, env, optional } = options;
  const shell = Deno.env.get("SHELL") || "/bin/bash";
  const mergedEnv = env ? { ...Deno.env.toObject(), ...env } : undefined;
  if (description) {
    console.log(`→ ${description}`);
  } else {
    console.log(`→ ${command}`);
  }
  let builder = commandRunner`${shell} -lc ${command}`
    .stdout("inherit")
    .stderr("inherit");
  if (cwd) builder = builder.cwd(cwd);
  if (mergedEnv) builder = builder.env(mergedEnv);
  const result = await builder.noThrow();
  if (result.code !== 0 && !optional) {
    throw new Error(`Command failed (${result.code ?? 1}): ${command}`);
  }
}

async function applyLinkPackages(
  action: LinkPackagesAction,
  ctx: ModuleContext,
): Promise<void> {
  const baseDir = resolvePath(ctx.moduleDir, action.base ?? "packages");
  const destDir = resolvePath(ctx.repoDir, action.destination ?? "src");
  await Deno.mkdir(destDir, { recursive: true });
  for (const pkg of action.packages) {
    const srcPath = resolvePath(baseDir, pkg);
    try {
      const info = await Deno.stat(srcPath);
      if (!info.isDirectory) {
        console.warn(
          `[module:${ctx.module}] link_packages: ${srcPath} is not a directory`,
        );
        continue;
      }
    } catch (err) {
      if (err instanceof Deno.errors.NotFound) {
        console.warn(
          `[module:${ctx.module}] Missing package directory: ${srcPath}`,
        );
        continue;
      }
      throw err;
    }
    const destPath = join(destDir, pkg);
    try {
      // Ensure parent dir for nested package names exists
      await Deno.mkdir(dirname(destPath), { recursive: true });
      // If destination already exists, skip linking
      try {
        const destInfo = await Deno.lstat(destPath);
        console.log(
          `[module:${ctx.module}] Destination already exists, skipping link: ${destPath}`,
        );
        continue;
      } catch (err) {
        if (!(err instanceof Deno.errors.NotFound)) throw err;
      }

      await Deno.symlink(srcPath, destPath);
      console.log(
        `[module:${ctx.module}] Linked package ${pkg} -> ${destPath}`,
      );
    } catch (err) {
      console.warn(
        `[module:${ctx.module}] Failed to link package ${pkg}: ${String(err)}`,
      );
    }
  }
}

async function applyGitClone(
  action: GitCloneAction,
  ctx: ModuleContext,
): Promise<void> {
  const baseDir = resolvePath(ctx.repoDir, action.base ?? "src");
  await Deno.mkdir(baseDir, { recursive: true });
  const destDir = resolvePath(baseDir, action.dest);
  try {
    const gitDir = resolvePath(destDir, ".git");
    await Deno.stat(gitDir);
    console.log(
      `[module:${ctx.module}] Repository already present: ${destDir}`,
    );
    return;
  } catch (err) {
    if (!(err instanceof Deno.errors.NotFound)) throw err;
  }
  const cloneArgs = ["clone"];
  if (action.branch) {
    cloneArgs.push("--branch", action.branch, "--single-branch");
  } else if (action.tag) {
    cloneArgs.push("--branch", action.tag, "--single-branch");
  }
  if (action.depth && action.depth > 0) {
    cloneArgs.push("--depth", String(action.depth));
  }
  if (action.sparse) {
    cloneArgs.push("--filter=blob:none");
  }
  cloneArgs.push(action.repo, destDir);
  console.log(`[module:${ctx.module}] Cloning ${action.repo} -> ${destDir}`);
  const result = await commandRunner`git ${cloneArgs}`
    .stdout("inherit")
    .stderr("inherit")
    .noThrow();
  if (result.code !== 0) {
    if ((action as ModuleActionBase).optional) {
      console.warn(
        `[module:${ctx.module}] Optional clone failed: ${action.repo}`,
      );
      return;
    }
    throw new Error(`git clone failed (${result.code ?? 1}): ${action.repo}`);
  }
}

async function applyAptInstall(
  action: AptInstallAction,
  ctx: ModuleContext,
): Promise<void> {
  // Use the user's shell to resolve `apt-get` (so shell builtins like `command`
  // are available). Some template runners execute commands directly which
  // means `command -v` (a shell builtin) can fail even though `apt-get` is
  // present on the filesystem. Fall back to a direct path existence check
  // if the shell check fails.
  const shell = Deno.env.get("SHELL") || "/bin/bash";
  const aptPath = await $`${shell} -lc 'command -v apt-get'`
    .stdout("piped").stderr("piped").noThrow();
  if (aptPath.code !== 0) {
    try {
      await Deno.stat("/usr/bin/apt-get");
    } catch (_err) {
      console.warn(
        `[module:${ctx.module}] apt-get not available; skipping packages: ${
          action.packages.join(", ")
        }`,
      );
      return;
    }
  }
  if (action.update) {
    console.log(colors.yellow(`[module:${ctx.module}] Running apt-get update`));
    // Stream apt output live while capturing it for later summary
    const upd = await runWithStreamingTee("sudo apt-get update");
    if (upd.code !== 0) {
      console.error(
        colors.red(
          `[module:${ctx.module}] apt-get update failed (code ${upd.code})`,
        ),
      );
      if (upd.stderr) console.error(colors.red(upd.stderr));
    }
    summarizeCommandOutput(upd.stdout, `[module:${ctx.module}] apt-get update`);
  }
  if (action.packages.length === 0) return;
  console.log(colors.yellow(
    `[module:${ctx.module}] Installing apt packages: ${
      action.packages.join(", ")
    }`,
  ));
  const pkgCmd = `sudo apt-get install -y ${action.packages.join(" ")}`;
  const result = await runWithStreamingTee(pkgCmd);
  if (result.code !== 0) {
    console.warn(colors.red(
      `[module:${ctx.module}] apt-get install exited with code ${result.code}`,
    ));
    if (result.stderr) console.error(colors.red(result.stderr));
  }
  summarizeCommandOutput(
    result.stdout,
    `[module:${ctx.module}] apt-get install`,
  );
}

function summarizeCommandOutput(text: string, label = "command") {
  try {
    const out = (text || "").replace(/\r\n/g, "\n").trim();
    if (!out) {
      console.log(colors.gray(`${label}: (no output)`));
      return;
    }
    const lines = out.split(/\n/);
    const head = 12;
    const tail = 6;
    if (lines.length <= head + tail + 3) {
      console.log(colors.gray(`${label}: output:`));
      console.log(out);
    } else {
      console.log(colors.gray(`${label}: output (first ${head} lines):`));
      console.log(lines.slice(0, head).join("\n"));
      console.log(
        colors.gray(`... (${lines.length - head - tail} lines omitted) ...`),
      );
      console.log(colors.gray(`${label}: output (last ${tail} lines):`));
      console.log(lines.slice(-tail).join("\n"));
    }
  } catch (_err) {
    // ignore summarization failures
  }
}

async function applyPipInstall(
  action: PipInstallAction,
  ctx: ModuleContext,
): Promise<void> {
  const python = action.python ?? "python3";
  if (action.import_check && action.import_check.length > 0) {
    const program = action.import_check.map((mod) => `import ${mod}`).join(
      "; ",
    );
    const check = await $`${python} -c ${program}`.noThrow();
    if (check.code === 0) {
      console.log(
        `[module:${ctx.module}] Python modules already installed: ${
          action.import_check.join(", ")
        }`,
      );
      return;
    }
  }
  // Build a safe, quoted command string so flags like
  // --break-system-packages are passed reliably (some template runners
  // do odd argument interpolation with arrays). Use runWithStreamingTee so
  // output is streamed and captured for diagnostics.
  const shellEscape = (s: string) => `'${s.replace(/'/g, "'\\''")}'`;
  const pkgList = action.packages.map(shellEscape).join(" ");
  const breakFlag = action.break_system ? "--break-system-packages" : "";
  const userFlag = action.user ? "--user" : "";
  const baseCmd = `${python} -m pip install ${pkgList} ${breakFlag} ${userFlag}`
    .replace(/\s+/g, " ").trim();
  console.log(
    `[module:${ctx.module}] Installing Python packages via ${python}: ${
      action.packages.join(", ")
    }`,
  );

  let result = await runWithStreamingTee(baseCmd);

  // Retry with sudo if non-root install failed and user didn't request --user
  if (result.code !== 0 && !action.user) {
    const sudoCheck = await $`command -v sudo`.noThrow();
    if (sudoCheck.code === 0) {
      console.log(`[module:${ctx.module}] Retrying pip install with sudo.`);
      // Use sudo to run the same command. Don't use -E by default; preserve
      // the python executable path by invoking the same ${python} command.
      result = await runWithStreamingTee(`sudo ${baseCmd}`);
    }
  }

  if (result.code !== 0) {
    console.warn(
      `[module:${ctx.module}] pip install exited with code ${result.code}`,
    );
    if (result.stderr) console.error(result.stderr);
  }

  // If import_check was provided, verify the modules are now importable and
  // warn if they're still missing.
  if (action.import_check && action.import_check.length > 0) {
    const program = action.import_check.map((mod) => `import ${mod}`).join(
      "; ",
    );
    const checkAfter = await $`${python} -c ${program}`.noThrow();
    if (checkAfter.code === 0) {
      console.log(
        `[module:${ctx.module}] Python modules now available: ${
          action.import_check.join(", ")
        }`,
      );
    } else {
      console.warn(
        `[module:${ctx.module}] After pip install, imports still failing: ${
          action.import_check.join(", ")
        }`,
      );
    }
  }
}

async function applyRosRun(
  action: RosRunAction,
  ctx: ModuleContext,
): Promise<void> {
  const args = action.args ?? [];
  await runCommand(
    `ros2 run ${action.package} ${action.executable} ${args.join(" ")}`,
    {
      description: action.description ??
        `ros2 run ${action.package} ${action.executable}`,
      cwd: ctx.repoDir,
      env: ctx.moduleConfigPath
        ? { PSH_MODULE_CONFIG: ctx.moduleConfigPath }
        : undefined,
      optional: action.optional,
    },
  );
}

async function applyRun(action: RunAction, ctx: ModuleContext): Promise<void> {
  const env = action.env ? { ...action.env } : {};
  if (ctx.moduleConfigPath && !env.PSH_MODULE_CONFIG) {
    env.PSH_MODULE_CONFIG = ctx.moduleConfigPath;
  }
  if (!env.MODULE_DIR) {
    env.MODULE_DIR = ctx.moduleDir;
  }

  let command = action.command?.trim();
  if (action.script) {
    const scriptPath = action.script.startsWith("/")
      ? action.script
      : resolvePath(ctx.moduleDir, action.script);
    command = `"${scriptPath}"`;
  }

  if (!command) {
    throw new Error(
      `[module:${ctx.module}] run action is missing both 'command' and 'script' entries`,
    );
  }

  await runCommand(command, {
    description: action.description,
    cwd: action.cwd ? resolvePath(ctx.repoDir, action.cwd) : ctx.repoDir,
    env,
    optional: action.optional,
  });
}

export async function applyModuleActions(
  actions: ModuleAction[] | undefined,
  ctx: ModuleContext,
): Promise<void> {
  if (!actions || actions.length === 0) return;
  for (const action of actions) {
    switch (action.type) {
      case "link_packages":
        await applyLinkPackages(action as LinkPackagesAction, ctx);
        break;
      case "git_clone":
        await applyGitClone(action as GitCloneAction, ctx);
        break;
      case "apt_install":
        await aptInstallHandler(action as AptInstallAction, ctx);
        break;
      case "pip_install":
        await pipInstallHandler(action as PipInstallAction, ctx);
        break;
      case "ros_run":
        await applyRosRun(action as RosRunAction, ctx);
        break;
      case "run":
        await applyRun(action as RunAction, ctx);
        break;
      default:
        console.warn(
          `[module:${ctx.module}] Unknown action type: ${
            (action as ModuleActionBase).type
          }`,
        );
    }
  }
}

interface AggregatedModuleContext {
  module: string;
  ctx: ModuleContext;
  spec: ModuleSpec;
  actions: ModuleAction[];
}

interface PipAggregate {
  python?: string;
  breakSystem: boolean;
  user: boolean;
  packages: string[];
  packageSet: Set<string>;
  importChecks: string[];
  importSet: Set<string>;
}

export async function setupMultipleModules(modules: string[]): Promise<void> {
  const ordered: string[] = [];
  const seen = new Set<string>();
  for (const name of modules) {
    const trimmed = name?.trim();
    if (!trimmed) continue;
    if (seen.has(trimmed)) continue;
    seen.add(trimmed);
    ordered.push(trimmed);
  }

  if (!ordered.length) return;

  const repoDir = repoDirFromModules();
  const moduleRoot = join(repoDir, "modules");
  const aggregateLabel = `multi:${ordered.join("+")}`;

  const contexts: AggregatedModuleContext[] = [];
  const aptPackages: string[] = [];
  const aptSeen = new Set<string>();
  let aptUpdate = false;

  const pipAggregates = new Map<string, PipAggregate>();

  const ensurePipAggregate = (action: PipInstallAction): PipAggregate => {
    const pythonKey = action.python ?? "";
    const key = `${pythonKey}|${action.break_system ? 1 : 0}|${action.user ? 1 : 0}`;
    let aggregate = pipAggregates.get(key);
    if (!aggregate) {
      aggregate = {
        python: action.python,
        breakSystem: Boolean(action.break_system),
        user: Boolean(action.user),
        packages: [],
        packageSet: new Set<string>(),
        importChecks: [],
        importSet: new Set<string>(),
      };
      pipAggregates.set(key, aggregate);
    }
    return aggregate;
  };

  try {
    for (const moduleName of ordered) {
      const specInfo = await loadModuleSpec(moduleName);
      if (!specInfo) {
        console.warn(`[module:${moduleName}] Module specification not found; skipping.`);
        continue;
      }
      const ctx = await prepareModuleContext(moduleName);
      contexts.push({
        module: moduleName,
        ctx,
        spec: specInfo.spec,
        actions: [],
      });
    }

    if (!contexts.length) return;

    for (const entry of contexts) {
      const moduleActions = entry.spec.actions ?? [];
      const remaining: ModuleAction[] = [];
      for (const action of moduleActions) {
        if (action.type === "apt_install") {
          const aptAction = action as AptInstallAction;
          aptUpdate ||= Boolean(aptAction.update);
          for (const pkg of aptAction.packages ?? []) {
            if (aptSeen.has(pkg)) continue;
            aptSeen.add(pkg);
            aptPackages.push(pkg);
          }
          continue;
        }
        if (action.type === "pip_install") {
          const pipAction = action as PipInstallAction;
          const aggregate = ensurePipAggregate(pipAction);
          for (const pkg of pipAction.packages ?? []) {
            if (aggregate.packageSet.has(pkg)) continue;
            aggregate.packageSet.add(pkg);
            aggregate.packages.push(pkg);
          }
          if (pipAction.import_check) {
            for (const modName of pipAction.import_check) {
              if (aggregate.importSet.has(modName)) continue;
              aggregate.importSet.add(modName);
              aggregate.importChecks.push(modName);
            }
          }
          continue;
        }
        remaining.push(action);
      }
      entry.actions = remaining;
    }

    const aggregateCtx: ModuleContext = {
      module: aggregateLabel,
      repoDir,
      moduleDir: moduleRoot,
      srcDir: join(repoDir, "src"),
      moduleConfig: null,
      moduleConfigPath: null,
      moduleConfigOwned: false,
    };

    if (aptPackages.length) {
      await aptInstallHandler({
        type: "apt_install",
        packages: aptPackages,
        update: aptUpdate,
      }, aggregateCtx);
    }

    for (const aggregate of pipAggregates.values()) {
      const pipAction: PipInstallAction = {
        type: "pip_install",
        packages: aggregate.packages,
      };
      if (aggregate.importChecks.length) {
        pipAction.import_check = aggregate.importChecks;
      }
      if (aggregate.python !== undefined) {
        pipAction.python = aggregate.python;
      }
      if (aggregate.breakSystem) {
        pipAction.break_system = true;
      }
      if (aggregate.user) {
        pipAction.user = true;
      }
      await pipInstallHandler(pipAction, aggregateCtx);
    }

    for (const entry of contexts) {
      if (!entry.actions.length) continue;
      await applyModuleActions(entry.actions, entry.ctx);
    }
  } finally {
    for (const entry of contexts) {
      await cleanupModuleContext(entry.ctx);
    }
  }
}

export function setAptInstallHandler(handler: AptInstallHandler): void {
  aptInstallHandler = handler;
}

export function resetAptInstallHandler(): void {
  aptInstallHandler = applyAptInstall;
}

export function setPipInstallHandler(handler: PipInstallHandler): void {
  pipInstallHandler = handler;
}

export function resetPipInstallHandler(): void {
  pipInstallHandler = applyPipInstall;
}

function isRecord(value: unknown): value is Record<string, unknown> {
  return Boolean(value) && typeof value === "object" && !Array.isArray(value);
}

function readToml(filePath: string): Promise<Record<string, unknown>> {
  return Deno.readTextFile(filePath).then((text) =>
    parseToml(text) as Record<string, unknown>
  );
}

export function repoDirFromModules(): string {
  // Always resolve repo root as parent of psh/ directory
  // This works even if cwd or import context is different
  // __dirname equivalent for Deno:
  const moduleFile = fromFileUrl(import.meta.url);
  const pshDir = dirname(moduleFile);
  // If this file is /home/pete/psyched/psh/modules.ts, repo root is /home/pete/psyched
  return dirname(pshDir);
}

export async function loadModuleSpec(
  moduleName: string,
): Promise<{ spec: ModuleSpec; path: string } | null> {
  const repoDir = repoDirFromModules();
  const moduleTomlPath = join(repoDir, "modules", moduleName, "module.toml");
  try {
    const spec = await readToml(moduleTomlPath) as ModuleSpec;
    return { spec, path: moduleTomlPath };
  } catch (err) {
    if (err instanceof Deno.errors.NotFound) {
      return null;
    }
    throw err;
  }
}

export interface ModuleConfigSource {
  path: string | null;
  data: Record<string, unknown> | null;
}

export interface PrepareModuleContextOptions {
  configData?: Record<string, unknown> | null;
  configPath?: string | null;
}

function configCandidatePaths(
  repoDir: string,
  host: string,
  module: string,
): string[] {
  const configs: string[] = [];
  const hostDir = join(repoDir, "hosts", host, "config");
  configs.push(join(hostDir, `${module}.yaml`));
  configs.push(join(hostDir, `${module}.yml`));
  const sharedDir = join(repoDir, "config");
  configs.push(join(sharedDir, `${module}.yaml`));
  configs.push(join(sharedDir, `${module}.yml`));
  return configs;
}

async function tryLoadYaml(
  path: string,
): Promise<Record<string, unknown> | null> {
  try {
    const stat = await Deno.stat(path);
    if (!stat.isFile) return null;
  } catch (err) {
    if (err instanceof Deno.errors.NotFound) return null;
    throw err;
  }
  try {
    const text = await Deno.readTextFile(path);
    const parsed = parseYaml(text);
    if (isRecord(parsed)) {
      return parsed as Record<string, unknown>;
    }
    console.warn(`[psh] Config ${path} is not a mapping; ignoring contents.`);
    return null;
  } catch (err) {
    console.warn(`[psh] Failed to read config ${path}: ${String(err)}`);
    return null;
  }
}

export async function loadHostModuleConfig(
  host: string,
  module: string,
): Promise<ModuleConfigSource> {
  const repoDir = repoDirFromModules();
  for (const candidate of configCandidatePaths(repoDir, host, module)) {
    const loaded = await tryLoadYaml(candidate);
    if (loaded === null) {
      continue;
    }
    return {
      path: candidate,
      data: Object.keys(loaded).length > 0 ? loaded : null,
    };
  }
  return { path: null, data: null };
}

export async function prepareModuleContext(
  module: string,
  options?: PrepareModuleContextOptions,
): Promise<ModuleContext> {
  const repoDir = repoDirFromModules();
  const moduleDir = join(repoDir, "modules", module);
  const srcDir = join(repoDir, "src");
  await Deno.mkdir(srcDir, { recursive: true });

  let moduleConfig = options?.configData ?? null;
  let moduleConfigPath = options?.configPath ?? null;
  let moduleConfigOwned = false;

  if (moduleConfigPath) {
    try {
      const stat = await Deno.stat(moduleConfigPath);
      if (!stat.isFile) {
        console.warn(
          `[module:${module}] Config path ${moduleConfigPath} is not a file; ignoring.`,
        );
        moduleConfigPath = null;
      }
    } catch (err) {
      if (err instanceof Deno.errors.NotFound) {
        console.warn(
          `[module:${module}] Config path ${moduleConfigPath} not found; ignoring.`,
        );
        moduleConfigPath = null;
      } else {
        throw err;
      }
    }
  }

  if (moduleConfigPath && !moduleConfig) {
    try {
      const text = await Deno.readTextFile(moduleConfigPath);
      const parsed = parseYaml(text);
      if (isRecord(parsed)) {
        moduleConfig = parsed as Record<string, unknown>;
      }
    } catch (err) {
      console.warn(
        `[module:${module}] Failed to parse config ${moduleConfigPath}: ${
          String(err)
        }`,
      );
    }
  }

  if (
    !moduleConfigPath && moduleConfig && Object.keys(moduleConfig).length > 0
  ) {
    const yaml = stringifyYaml(moduleConfig as Record<string, unknown>);
    const tempDir = Deno.makeTempDirSync({ prefix: `psh_${module}_` });
    moduleConfigPath = join(tempDir, "params.yaml");
    await Deno.writeTextFile(moduleConfigPath, yaml);
    moduleConfigOwned = true;
  }

  return {
    module,
    repoDir,
    moduleDir,
    srcDir,
    moduleConfig,
    moduleConfigPath,
    moduleConfigOwned,
  };
}

export async function cleanupModuleContext(ctx: ModuleContext): Promise<void> {
  if (ctx.moduleConfigOwned && ctx.moduleConfigPath) {
    try {
      const parent = dirname(ctx.moduleConfigPath);
      await Deno.remove(parent, { recursive: true });
    } catch { /* ignore */ }
  }
}
