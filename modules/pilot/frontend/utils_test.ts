import { assertStrictEquals } from "$std/assert/assert_strict_equals.ts";
import { assertEquals } from "$std/assert/assert_equals.ts";

import { define, type State } from "./utils.ts";

Deno.test("define.page returns the supplied component", () => {
  const component = () => "ok";
  const result = define.page(component);
  assertStrictEquals(result, component);
});

Deno.test("define.handlers preserves handler references", () => {
  const handlers = {
    async POST() {
      return new Response("pong");
    },
  } satisfies Parameters<typeof define.handlers>[0];

  const result = define.handlers(handlers);
  assertStrictEquals(result, handlers);
});

Deno.test("state type allows optional build metadata", () => {
  const state: State = { buildInfo: { version: "1.2.3" } };
  assertEquals(state.buildInfo?.version, "1.2.3");
});
