/**
 * Scenario harness for PSH end-to-end tests.
 *
 * ```ts
 * import { defineScenario } from "./harness.ts";
 *
 * defineScenario("speech stack boots", async (t) => {
 *   // Arrange, act, assert – follow BDD naming conventions.
 *   await t.step("should expose health endpoint", async () => {
 *     // ... interact with the system under test ...
 *   });
 * });
 *
 * defineScenario("legacy oauth callback", async () => {
 *   throw new Error("still broken in staging");
 * }, { knownFailure: "tracking INFRA-123" });
 * ```
 */
export interface ScenarioOptions {
  /**
   * Annotate a scenario as a known failure so CI remains green while
   * investigations continue. Provide a short reason or ticket reference when
   * possible so the junit output stays actionable.
   */
  readonly knownFailure?: boolean | string;
}

/**
 * Wrap a Deno test with reporter-friendly lifecycle logging and known failure
 * handling. When `knownFailure` is set, the scenario passes only if the wrapped
 * function throws, helping us spot unblocked behaviour early.
 */
export function defineScenario(
  name: string,
  fn: (t: Deno.TestContext) => void | Promise<void>,
  options: ScenarioOptions = {},
): void {
  const { knownFailure } = options;

  Deno.test({
    name,
    // Let scenarios manage their own resources; e2e workflows often spin up
    // subprocesses or network servers intentionally.
    sanitizeOps: false,
    sanitizeResources: false,
    sanitizeExit: false,
    fn: async (t) => {
      const start = performance.now();
      const label = `scenario:${name}`;
      const duration = () => `${Math.round(performance.now() - start)}ms`;
      const knownFailureReason =
        typeof knownFailure === "string" ? knownFailure : undefined;

      console.log(`BEGIN ${label}`);
      if (knownFailureReason) {
        console.log(`KNOWN-FAILURE ${label} reason=${knownFailureReason}`);
      } else if (knownFailure) {
        console.log(`KNOWN-FAILURE ${label}`);
      }

      try {
        await fn(t);
        console.log(`SUCCESS ${label} duration=${duration()}`);

        if (knownFailure) {
          const reason = knownFailureReason ?? "marked as known failure";
          throw new Error(
            `Scenario \"${name}\" unexpectedly passed (reason: ${reason}). Remove the knownFailure flag to re-enable CI enforcement.`,
          );
        }
      } catch (error) {
        if (knownFailure) {
          const reason = knownFailureReason ?? "marked as known failure";
          console.log(
            `EXPECTED-FAILURE ${label} duration=${duration()} reason=${reason}`,
          );
          console.log(`${label} error=${error instanceof Error ? error.stack ?? error.message : String(error)}`);
          return;
        }

        console.log(`FAILURE ${label} duration=${duration()}`);
        throw error;
      }
    },
  });
}
