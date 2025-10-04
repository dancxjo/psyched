import { assertEquals, assertThrows } from "$std/testing/asserts.ts";
import { applySudo } from "./util.ts";

Deno.test("applySudo keeps args unchanged for root", () => {
  const result = applySudo(["apt-get", "update"], {
    requireSudo: true,
    isRoot: true,
    sudoPath: undefined,
  });
  assertEquals(result, ["apt-get", "update"]);
});

Deno.test("applySudo prefixes sudo when available", () => {
  const result = applySudo(["apt-get", "update"], {
    requireSudo: true,
    isRoot: false,
    sudoPath: "/usr/bin/sudo",
  });
  assertEquals(result, ["/usr/bin/sudo", "apt-get", "update"]);
});

Deno.test("applySudo throws when required but unavailable", () => {
  assertThrows(
    () => {
      applySudo(["apt-get", "update"], {
        requireSudo: true,
        isRoot: false,
        sudoPath: undefined,
      });
    },
    Error,
    "sudo is required",
  );
});

Deno.test("applySudo leaves args when sudo not required", () => {
  const result = applySudo(["apt-get", "update"], {
    requireSudo: false,
    isRoot: false,
    sudoPath: undefined,
  });
  assertEquals(result, ["apt-get", "update"]);
});
