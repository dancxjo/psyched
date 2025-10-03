import { assert, assertArrayIncludes } from "$std/testing/asserts.ts";
import { listServices } from "./service.ts";

Deno.test("listServices finds declared services", () => {
  const services = listServices();
  assert(services.length > 0, "expected at least one service");
  assertArrayIncludes(services, ["tts"], "tts service should be present");
});
