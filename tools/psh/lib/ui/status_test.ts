import {
  assert,
  assertRejects,
  assertStringIncludes,
} from "$std/testing/asserts.ts";
import { StatusReporter } from "./status.ts";

function capture() {
  const lines: string[] = [];
  return {
    lines,
    lineWriter: (text: string) => {
      lines.push(text);
    },
  };
}

Deno.test("StatusReporter prints success summary", async () => {
  const { lines, lineWriter } = capture();
  const reporter = new StatusReporter({ verbose: true, lineWriter });
  await reporter.step("Install Deno", (step) => {
    step.log("checking version");
  });
  const header = lines.find((line) => line.includes("Install Deno"));
  assert(header, "expected header line to reference Install Deno");
  const success = lines.find((line) => line.includes("✔"));
  assert(success, "expected success line");
  assertStringIncludes(success, "Install Deno");
});

Deno.test("StatusReporter surfaces error details", async () => {
  const { lines, lineWriter } = capture();
  const reporter = new StatusReporter({ verbose: true, lineWriter });
  await assertRejects(async () => {
    await reporter.step("Failing step", () => {
      throw new Error("boom");
    });
  });
  const joined = lines.join("\n");
  assertStringIncludes(joined, "Failing step");
  assertStringIncludes(joined, "boom");
});
