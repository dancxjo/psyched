import { assert, assertArrayIncludes } from "$std/testing/asserts.ts";
import { listModules, moduleStatuses } from "./module.ts";

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
