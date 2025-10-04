import {
  assertEquals,
  assertThrows,
} from "$std/testing/asserts.ts";
import { resolveTargetBatches } from "./target_resolver.ts";

denoTest("returns all targets when none specified", () => {
  const catalog = { modules: ["alpha", "shared"], services: ["beta", "shared"] };
  const result = resolveTargetBatches([], { catalog });
  assertEquals(result.modules, ["alpha", "shared"]);
  assertEquals(result.services, ["beta", "shared"]);
});

function denoTest(name: string, fn: () => void | Promise<void>) {
  Deno.test(name, fn);
}

denoTest("prefers modules when names overlap by default", () => {
  const catalog = { modules: ["alpha", "shared"], services: ["beta", "shared"] };
  const result = resolveTargetBatches(["shared"], { catalog });
  assertEquals(result.modules, ["shared"]);
  assertEquals(result.services, []);
});

denoTest("can prefer services when flag enabled", () => {
  const catalog = { modules: ["alpha", "shared"], services: ["beta", "shared"] };
  const result = resolveTargetBatches(["shared"], {
    catalog,
    preferService: true,
  });
  assertEquals(result.modules, []);
  assertEquals(result.services, ["shared"]);
});

denoTest("deduplicates targets while preserving order", () => {
  const catalog = { modules: ["alpha", "gamma"], services: ["beta"] };
  const result = resolveTargetBatches(["alpha", "alpha", "beta", "alpha"], {
    catalog,
  });
  assertEquals(result.modules, ["alpha"]);
  assertEquals(result.services, ["beta"]);
});

denoTest("throws helpful error for unknown target", () => {
  const catalog = { modules: ["alpha"], services: ["beta"] };
  assertThrows(
    () => resolveTargetBatches(["ghost"], { catalog }),
    Error,
    "Unknown target 'ghost'",
  );
});
