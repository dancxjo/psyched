import { CommandBuilder } from "$dax";
import { StatusReporter, StepHandle } from "../ui/status.ts";
import { applySudo } from "./util.ts";

export interface ExecOptions {
  sudo?: boolean;
  env?: Record<string, string>;
  cwd?: string;
  description?: string;
  stdoutOnSuccess?: boolean;
  stderrOnSuccess?: boolean;
  timeoutMs?: number;
}

export interface ProvisionContextOptions {
  verbose?: boolean;
  showLogsOnSuccess?: boolean;
  reporter?: StatusReporter;
  system?: {
    isRoot?: boolean;
    sudoPath?: string;
  };
  config?: Record<string, unknown>;
}

export class ProvisionContext {
  readonly reporter: StatusReporter;
  readonly verbose: boolean;
  readonly isRoot: boolean;
  readonly sudoPath?: string;
  readonly config: Record<string, unknown>;

  constructor(options: ProvisionContextOptions = {}) {
    this.verbose = options.verbose ?? false;
    this.reporter = options.reporter ?? new StatusReporter({
      verbose: this.verbose,
      showLogsOnSuccess: options.showLogsOnSuccess,
    });
    this.isRoot = options.system?.isRoot ?? this.#detectRoot();
    this.sudoPath = options.system?.sudoPath ??
      (this.isRoot ? undefined : this.#detectSudo());
    this.config = options.config ? { ...options.config } : {};
  }

  async step<T>(
    title: string,
    action: (step: StepContext) => Promise<T> | T,
  ): Promise<T> {
    return await this.reporter.step(title, (handle) => {
      const context = new StepContext(this, handle);
      return action(context);
    });
  }

  async exec(
    handle: StepHandle,
    args: string[],
    options: ExecOptions = {},
  ): Promise<CommandResultWithOutput> {
    const finalArgs = applySudo(args, {
      requireSudo: options.sudo ?? false,
      isRoot: this.isRoot,
      sudoPath: this.sudoPath,
    });
    const description = options.description ?? args.join(" ");
    if (this.verbose) {
      handle.log(`$ ${finalArgs.join(" ")}`);
    }

    let builder = new CommandBuilder().command(finalArgs)
      .stdout(this.verbose ? "inheritPiped" : "piped")
      .stderr(this.verbose ? "inheritPiped" : "piped")
      .noThrow();
    if (options.env) builder = builder.env(options.env);
    if (options.cwd) builder = builder.cwd(options.cwd);
    if (options.timeoutMs) builder = builder.timeout(options.timeoutMs);

    const result = await builder.spawn();
    if (result.code !== 0) {
      handle.output("stdout", result.stdout);
      handle.output("stderr", result.stderr);
      throw new Error(`${description} failed with exit code ${result.code}`);
    }
    if (options.stdoutOnSuccess && result.stdout.trim()) {
      handle.output("stdout", result.stdout);
    }
    if (options.stderrOnSuccess && result.stderr.trim()) {
      handle.output("stderr", result.stderr);
    }
    return {
      code: result.code,
      stdout: result.stdout,
      stderr: result.stderr,
    };
  }

  #detectRoot(): boolean {
    try {
      return typeof (Deno as unknown as { uid?: () => number }).uid ===
          "function"
        ? ((Deno as unknown as { uid: () => number }).uid() === 0)
        : false;
    } catch {
      return false;
    }
  }

  #detectSudo(): string | undefined {
    try {
      const result = new Deno.Command("which", {
        args: ["sudo"],
        stdout: "piped",
        stderr: "null",
      }).outputSync();
      if (result.success) {
        return new TextDecoder().decode(result.stdout).trim();
      }
    } catch {
      // ignore
    }
    return undefined;
  }
}

export interface CommandResultWithOutput {
  code: number;
  stdout: string;
  stderr: string;
}

export class StepContext {
  #parent: ProvisionContext;
  #handle: StepHandle;

  constructor(parent: ProvisionContext, handle: StepHandle) {
    this.#parent = parent;
    this.#handle = handle;
  }

  get verbose(): boolean {
    return this.#handle.verbose;
  }

  log(message: string): void {
    this.#handle.log(message);
  }

  output(kind: "stdout" | "stderr", message: string): void {
    this.#handle.output(kind, message);
  }

  exec(
    args: string[],
    options: ExecOptions = {},
  ): Promise<CommandResultWithOutput> {
    return this.#parent.exec(this.#handle, args, options);
  }
}
