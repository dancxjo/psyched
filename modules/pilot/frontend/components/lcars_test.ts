import { describe, it } from "$std/testing/bdd.ts";
import { assertEquals } from "$std/testing/asserts.ts";

import { toneFromConnection } from "./lcars.tsx";

describe("toneFromConnection", () => {
  it("maps websocket statuses to badge tones", () => {
    assertEquals(toneFromConnection("open"), "ok");
    assertEquals(toneFromConnection("connecting"), "info");
    assertEquals(toneFromConnection("closed"), "warn");
    assertEquals(toneFromConnection("error"), "danger");
    assertEquals(toneFromConnection("idle"), "neutral");
  });

  it("falls back to neutral for unknown states", () => {
    assertEquals(toneFromConnection(undefined), "neutral");
  });
});
