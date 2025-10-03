import { colors } from "$cliffy/ansi/colors.ts";

export interface StepHandle {
  readonly verbose: boolean;
  log(message: string): void;
  output(kind: "stdout" | "stderr", message: string): void;
}

export interface StatusReporterOptions {
  verbose?: boolean;
  showLogsOnSuccess?: boolean;
  /** Custom writer used when rendering spinner frames (defaults to stdout). */
  streamWriter?: (text: string) => void;
  /** Custom writer used when printing full lines (defaults to console.log). */
  lineWriter?: (text: string) => void;
  /** Number of milliseconds between spinner frame updates. */
  spinnerIntervalMs?: number;
}

const frames = ["⠋", "⠙", "⠹", "⠸", "⠼", "⠴", "⠦", "⠧", "⠇", "⠏"];
const encoder = new TextEncoder();

export class StatusReporter {
  #verbose: boolean;
  #showLogsOnSuccess: boolean;
  #streamWriter: (text: string) => void;
  #lineWriter: (text: string) => void;
  #spinnerInterval: number;

  constructor(options: StatusReporterOptions = {}) {
    this.#verbose = options.verbose ?? false;
    this.#showLogsOnSuccess = options.showLogsOnSuccess ?? false;
    this.#streamWriter = options.streamWriter ?? ((text) => {
      Deno.stdout.writeSync(encoder.encode(text));
    });
    this.#lineWriter = options.lineWriter ?? ((text) => {
      console.log(text);
    });
    this.#spinnerInterval = options.spinnerIntervalMs ?? 120;
  }

  get verbose(): boolean {
    return this.#verbose;
  }

  async step<T>(
    title: string,
    action: (handle: StepHandle) => Promise<T> | T,
  ): Promise<T> {
    const logs: string[] = [];
    const outputs: Array<["stdout" | "stderr", string]> = [];
    const handle: StepHandle = {
      verbose: this.#verbose,
      log: (message: string) => {
        logs.push(message);
        if (this.#verbose) {
          this.#lineWriter(`  ${colors.gray(message)}`);
        }
      },
      output: (kind: "stdout" | "stderr", message: string) => {
        outputs.push([kind, message]);
        if (this.#verbose) {
          const color = kind === "stdout" ? colors.cyan : colors.yellow;
          this.#lineWriter(`  ${color(`[${kind}]`)} ${message}`);
        }
      },
    };

    if (this.#verbose) {
      this.#lineWriter(`${colors.cyan("▶")} ${colors.bold(title)}`);
    }

    let timer: number | undefined;
    let active = false;
    let frameIndex = 0;

    if (!this.#verbose && this.#isInteractive()) {
      active = true;
      const render = () => {
        const frame = frames[frameIndex];
        frameIndex = (frameIndex + 1) % frames.length;
        this.#streamWriter(`\r${colors.cyan(frame)} ${title}`);
      };
      render();
      timer = setInterval(render, this.#spinnerInterval);
    } else if (!this.#verbose) {
      this.#lineWriter(`${colors.cyan("…")} ${title}`);
    }

    try {
      const result = await action(handle);
      this.#finishStep({
        title,
        logs,
        outputs,
        success: true,
        active,
        timer,
      });
      return result;
    } catch (error) {
      this.#finishStep({
        title,
        logs,
        outputs,
        success: false,
        active,
        timer,
        error,
      });
      throw error;
    }
  }

  #finishStep(args: {
    title: string;
    logs: string[];
    outputs: Array<["stdout" | "stderr", string]>;
    success: boolean;
    active: boolean;
    timer?: number;
    error?: unknown;
  }): void {
    if (args.timer !== undefined) {
      clearInterval(args.timer);
    }
    if (!this.#verbose && args.active) {
      const clear = " ".repeat(args.title.length + 4);
      this.#streamWriter(`\r${clear}\r`);
    }

    if (args.success) {
      const symbol = colors.green("✔");
      this.#lineWriter(`${symbol} ${args.title}`);
      if (this.#verbose || this.#showLogsOnSuccess) {
        this.#flushLogs(args.logs, args.outputs);
      }
    } else {
      const symbol = colors.red("✖");
      this.#lineWriter(`${symbol} ${args.title}`);
      this.#flushLogs(args.logs, args.outputs);
      if (args.error) {
        const message = args.error instanceof Error
          ? args.error.message
          : String(args.error);
        this.#lineWriter(colors.red(`  ↳ ${message}`));
      }
    }
  }

  #flushLogs(
    logs: string[],
    outputs: Array<["stdout" | "stderr", string]>,
  ): void {
    for (const log of logs) {
      this.#lineWriter(`  ${colors.gray(log)}`);
    }
    for (const [kind, message] of outputs) {
      const color = kind === "stdout" ? colors.cyan : colors.yellow;
      const trimmed = message.trim();
      if (!trimmed) continue;
      this.#lineWriter(
        `  ${color(`[${kind}]`)} ${trimmed.replaceAll("\n", "\n    ")}`,
      );
    }
  }

  #isInteractive(): boolean {
    try {
      const stdout = Deno.stdout as { isTerminal?: () => boolean };
      if (typeof stdout.isTerminal === "function") {
        return stdout.isTerminal();
      }
    } catch {
      // ignore, treat as non-interactive
    }
    return false;
  }
}
