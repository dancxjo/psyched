import { parse as parseToml } from "@std/toml";
import { stringify as stringifyYaml } from "@std/yaml";
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

  /** Reset the command runner back to the default Dax tag. */
  function resetModuleCommandRunner(): void {
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
  }

  function resolvePath(base: string, rel?: string | null): string {
    if (!rel) return base;
    if (rel.startsWith("/")) return rel;
    return join(base, rel);
  }

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
    console.log(`[module:${ctx.module}] Skipped linking package ${pkg}`);
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
        `[module:${ctx.module}] apt-get not available; skipping packages: ${action.packages.join(", ")
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
      console.error(colors.red(`[module:${ctx.module}] apt-get update failed (code ${upd.code})`));
      if (upd.stderr) console.error(colors.red(upd.stderr));
    }
    summarizeCommandOutput(upd.stdout, `[module:${ctx.module}] apt-get update`);
  }
  if (action.packages.length === 0) return;
  console.log(colors.yellow(
    `[module:${ctx.module}] Installing apt packages: ${action.packages.join(", ")}`,
  ));
  const pkgCmd = `sudo apt-get install -y ${action.packages.join(" ")}`;
  const result = await runWithStreamingTee(pkgCmd);
  if (result.code !== 0) {
    console.warn(colors.red(
      `[module:${ctx.module}] apt-get install exited with code ${result.code}`,
    ));
    if (result.stderr) console.error(colors.red(result.stderr));
  }
  summarizeCommandOutput(result.stdout, `[module:${ctx.module}] apt-get install`);
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
      console.log(colors.gray(`... (${lines.length - head - tail} lines omitted) ...`));
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
        `[module:${ctx.module}] Python modules already installed: ${action.import_check.join(", ")
        }`,
      );
      return;
    }
  }
  const pipArgs = ["-m", "pip", "install", ...action.packages];
  if (action.break_system) pipArgs.push("--break-system-packages");
  if (action.user) pipArgs.push("--user");
  console.log(
    `[module:${ctx.module}] Installing Python packages via ${python}: ${action.packages.join(", ")
    }`,
  );
  let result = await $`${python} ${pipArgs}`.noThrow();
  if (result.code !== 0 && !action.user) {
    const sudo = await $`command -v sudo`.noThrow();
    if (sudo.code === 0) {
      console.log(`[module:${ctx.module}] Retrying pip install with sudo.`);
      result = await $`sudo ${python} ${pipArgs}`.noThrow();
    }
  }
  if (result.code !== 0) {
    console.warn(
      `[module:${ctx.module}] pip install exited with code ${result.code}`,
    );
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
        await applyAptInstall(action as AptInstallAction, ctx);
        break;
      case "pip_install":
        await applyPipInstall(action as PipInstallAction, ctx);
        break;
      case "ros_run":
        await applyRosRun(action as RosRunAction, ctx);
        break;
      case "run":
        await applyRun(action as RunAction, ctx);
        break;
      default:
        console.warn(
          `[module:${ctx.module}] Unknown action type: ${(action as ModuleActionBase).type
          }`,
        );
    }
  }
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

export async function prepareModuleContext(
  module: string,
  moduleConfig: Record<string, unknown> | null,
): Promise<ModuleContext> {
  const repoDir = repoDirFromModules();
  const moduleDir = join(repoDir, "modules", module);
  const srcDir = join(repoDir, "src");
  await Deno.mkdir(srcDir, { recursive: true });
  let moduleConfigPath: string | null = null;
  if (moduleConfig && Object.keys(moduleConfig).length > 0) {
    const yaml = stringifyYaml(moduleConfig as Record<string, unknown>);
    moduleConfigPath = join(
      Deno.makeTempDirSync({ prefix: `psh_${module}_` }),
      "params.yaml",
    );
    await Deno.writeTextFile(moduleConfigPath, yaml);
  }
  return { module, repoDir, moduleDir, srcDir, moduleConfig, moduleConfigPath };
}

export async function cleanupModuleContext(ctx: ModuleContext): Promise<void> {
  if (ctx.moduleConfigPath) {
    try {
      const parent = dirname(ctx.moduleConfigPath);
      await Deno.remove(parent, { recursive: true });
    } catch { /* ignore */ }
  }
}
