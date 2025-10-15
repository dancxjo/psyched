import { assertEquals } from "$std/testing/asserts.ts";
import { __test__ } from "./systemd.ts";

const {
  environmentLines,
  mergeEnv,
  sanitizeEnvRecord,
  moduleUnitName,
  serviceUnitName,
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
