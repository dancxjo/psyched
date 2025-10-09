import {
  assert,
  assertArrayIncludes,
  assertEquals,
  assertRejects,
  assertStringIncludes,
} from "$std/testing/asserts.ts";
import { join } from "$std/path/mod.ts";
import { colors } from "$cliffy/ansi/colors.ts";
import {
  awaitModuleStability,
  bringModulesUp,
  composeLaunchCommand,
  formatExitSummary,
  formatLaunchDiagnostics,
  listModules,
  moduleStatuses,
} from "./module.ts";
import { repoRoot } from "./paths.ts";

Deno.test("listModules discovers known modules", () => {
  const modules = listModules();
  assert(modules.length > 0, "expected at least one module");
  assertArrayIncludes(modules, ["pilot"], "pilot module should be present");
});

Deno.test("moduleStatuses reports state for each module", () => {
  const statuses = moduleStatuses();
  const names = statuses.map((status) => status.name);
  assertArrayIncludes(names, ["pilot"], "pilot module should be listed");
  for (const status of statuses) {
    assert(["running", "stopped"].includes(status.status));
    if (status.status === "running") {
      assert(typeof status.pid === "number");
    }
  }
});

Deno.test("composeLaunchCommand directs streams through the prefix helper", () => {
  const command = composeLaunchCommand({
    envCommands: ["source /tmp/env"],
    launchScript: "/tmp/launch.sh",
    logFile: "/var/log/demo module.log",
    module: "demo module",
  });
  const prefixScript = join(
    repoRoot(),
    "tools",
    "psh",
    "scripts",
    "prefix_logs.sh",
  );
  assertStringIncludes(
    command,
    "source /tmp/env && exec bash '/tmp/launch.sh'",
  );
  assertStringIncludes(
    command,
    `> >('${prefixScript}' 'demo module' 'stdout' | tee -a '/var/log/demo module.log')`,
  );
  assertStringIncludes(
    command,
    `2> >('${prefixScript}' 'demo module' 'stderr' | tee -a '/var/log/demo module.log' >&2)`,
  );
});

Deno.test("composeLaunchCommand escapes single quotes", () => {
  const command = composeLaunchCommand({
    envCommands: [],
    launchScript: "/tmp/launch.sh",
    logFile: "/var/log/demo.log",
    module: "rocky's rig",
  });
  assertStringIncludes(command, `'rocky'"'"'s rig'`);
});

Deno.test("formatLaunchDiagnostics summarises the launch plan", () => {
  const lines = formatLaunchDiagnostics({
    module: "imu",
    launchScript: "/tmp/launch.sh",
    envCommands: ["source /env", "psyched::activate --quiet"],
    logFile: "/logs/imu.log",
    command: "exec launch --test",
  });
  assertEquals(lines, [
    "module: imu",
    "launch script: /tmp/launch.sh",
    "log file: /logs/imu.log",
    "environment bootstrap: source /env && psyched::activate --quiet",
    "spawn command: exec launch --test",
  ]);
});

Deno.test("formatExitSummary highlights success and failure", () => {
  const success = colors.stripColor(
    formatExitSummary("imu", { success: true, code: 0, signal: null }),
  );
  assertStringIncludes(success, "exited cleanly (code 0)");

  const failure = colors.stripColor(
    formatExitSummary("imu", {
      success: false,
      code: 1,
      signal: "SIGTERM",
    }),
  );
  assertStringIncludes(failure, "exited with code 1 (signal SIGTERM)");
});

Deno.test(
  "awaitModuleStability reports early exit when the process stops immediately",
  async () => {
    const status: Deno.CommandStatus = { success: false, code: 1, signal: null };
    const result = await awaitModuleStability(Promise.resolve(status), 5);
    assertEquals(result, status);
  },
);

Deno.test(
  "awaitModuleStability returns null when the process keeps running",
  async () => {
    const pending = new Promise<Deno.CommandStatus>(() => {});
    const result = await awaitModuleStability(pending, 5);
    assertEquals(result, null);
  },
);

Deno.test(
  "bringModulesUp continues launching remaining modules when one fails",
  async () => {
    const modules = ["faces", "nav", "gps"];
    const launched: string[] = [];
    const launcher = async (module: string) => {
      launched.push(module);
      if (module === "nav") {
        throw new Error("nav is unavailable");
      }
    };

    const error = await assertRejects(async () => {
      await bringModulesUp(modules, { launcher });
    });

    assert(error instanceof AggregateError);
    assertEquals(launched, modules);
    assertEquals(error.errors.length, 1);
    assertStringIncludes(error.message, "nav");
  },
);
