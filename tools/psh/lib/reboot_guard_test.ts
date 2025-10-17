import { assertEquals, assertThrows } from "$std/testing/asserts.ts";
import { ensureRebootCompleted, RebootRequiredError } from "./reboot_guard.ts";

Deno.test("guard passes when sentinel is missing", () => {
  ensureRebootCompleted({
    sentinelPath: `${Deno.makeTempDirSync()}/absent`,
    readBootTime: () => 123,
  });
});

Deno.test("guard blocks when reboot has not happened", () => {
  const dir = Deno.makeTempDirSync();
  const sentinel = `${dir}/reboot-required`;
  Deno.writeTextFileSync(sentinel, "123\n");
  assertThrows(
    () =>
      ensureRebootCompleted({
        sentinelPath: sentinel,
        readBootTime: () => 123,
      }),
    RebootRequiredError,
    "reboot",
  );
  assertEquals(Deno.readTextFileSync(sentinel).trim(), "123");
});

Deno.test("guard clears sentinel once rebooted", () => {
  const dir = Deno.makeTempDirSync();
  const sentinel = `${dir}/reboot-required`;
  Deno.writeTextFileSync(sentinel, "123\n");
  ensureRebootCompleted({
    sentinelPath: sentinel,
    readBootTime: () => 456,
  });
  assertThrows(() => Deno.statSync(sentinel));
});
