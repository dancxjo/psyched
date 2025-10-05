import {
  assert,
  assertArrayIncludes,
  assertStringIncludes,
} from "$std/testing/asserts.ts";
import { join } from "$std/path/mod.ts";
import {
  composeLaunchCommand,
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

Deno.test("composeLaunchCommand uses prefixed tee logging", () => {
  const command = composeLaunchCommand({
    envCommands: ["source /tmp/env"],
    launchScript: "/tmp/launch.sh",
    logFile: "/var/log/demo module.log",
    module: "demo module",
  });
  const prefixScript = join(repoRoot(), "tools", "psh", "scripts", "prefix_logs.sh");
  assertStringIncludes(command, "source /tmp/env && exec bash '/tmp/launch.sh'");
  assertStringIncludes(command, `> >('${prefixScript}' 'demo module' 'stdout' | tee -a '/var/log/demo module.log')`);
  assertStringIncludes(command, `2> >('${prefixScript}' 'demo module' 'stderr' | tee -a '/var/log/demo module.log' >&2)`);
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
