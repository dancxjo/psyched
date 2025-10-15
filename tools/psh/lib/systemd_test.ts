import { assertEquals } from "$std/testing/asserts.ts";
import { __test__ } from "./systemd.ts";

const {
  environmentLines,
  mergeEnv,
  sanitizeEnvRecord,
  moduleUnitName,
  serviceUnitName,
  isLingerEnabled,
  enableLinger,
} = __test__;

Deno.test("environmentLines sorts keys", () => {
  const lines = environmentLines({ B: "2", A: "1" });
  assertEquals(lines, ["Environment=A=1", "Environment=B=2"]);
});

Deno.test("mergeEnv combines sources with later overrides", () => {
  const env = mergeEnv({ A: "1" }, { B: "2" }, { A: "3" });
  assertEquals(env, { A: "3", B: "2" });
});

Deno.test("sanitizeEnvRecord stringifies non-string values", () => {
  const env = sanitizeEnvRecord({
    NUMERIC: 42,
    TRUTHY: true,
    EMPTY: "",
    NIL: null,
  });
  assertEquals(env, {
    NUMERIC: "42",
    TRUTHY: "true",
    EMPTY: "",
  });
});

Deno.test("unit name helpers add prefixes", () => {
  assertEquals(moduleUnitName("pilot"), "psh-module-pilot.service");
  assertEquals(serviceUnitName("tts"), "psh-service-tts.service");
});

Deno.test("isLingerEnabled checks loginctl output", async () => {
  // This is an integration test that checks the actual system state
  // It should not fail if loginctl is not available
  const result = await isLingerEnabled();
  // Just verify it returns a boolean without throwing
  assertEquals(typeof result, "boolean");
});

Deno.test("enableLinger can be called without error", async () => {
  // This test verifies that enableLinger handles missing USER gracefully
  const originalUser = Deno.env.get("USER");
  try {
    // Test with USER unset
    Deno.env.delete("USER");
    await enableLinger(); // Should not throw, just warn
  } finally {
    // Restore USER if it was set
    if (originalUser) {
      Deno.env.set("USER", originalUser);
    }
  }
});
