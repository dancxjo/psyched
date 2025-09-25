import type { DaxTemplateTag } from "./util.ts";

export interface StubInvocation {
  parts: TemplateStringsArray;
  values: unknown[];
  options: {
    stdout?: string;
    stderr?: string;
    cwd?: string;
    env?: Record<string, string>;
  };
}

export interface StubResult {
  code: number;
  stdout?: string;
  stderr?: string;
}

export type InvocationHandler = (
  invocation: StubInvocation,
) => StubResult | Promise<StubResult>;

export function createDaxStub(
  handler: InvocationHandler,
): DaxTemplateTag {
  return ((parts: TemplateStringsArray, ...values: unknown[]) => {
    const options: StubInvocation["options"] = {};
    const builder = {
      stdout(mode: string) {
        options.stdout = mode;
        return builder;
      },
      stderr(mode: string) {
        options.stderr = mode;
        return builder;
      },
      cwd(dir: string) {
        options.cwd = dir;
        return builder;
      },
      env(envVars: Record<string, string>) {
        options.env = envVars;
        return builder;
      },
      async noThrow() {
        const result = await handler({ parts, values, options });
        return {
          code: result.code,
          stdout: result.stdout ?? "",
          stderr: result.stderr ?? "",
        };
      },
    };
    return builder as unknown as ReturnType<DaxTemplateTag>;
  }) as DaxTemplateTag;
}
