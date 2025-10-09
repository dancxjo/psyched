import { assertEquals } from "$std/assert/mod.ts";

import {
  determineEnabledModules,
  determineEnabledServices,
} from "./host_config.ts";

Deno.test("determineEnabledModules only includes host declared modules", () => {
  const raw = {
    host: { modules: ["pilot", "imu"] },
    modules: {
      imu: { launch: true },
      foot: { launch: true },
      nav: { launch: { enabled: false } },
      eye: { launch: { arguments: { enabled: true } } },
    },
  } as Record<string, unknown>;

  assertEquals(
    determineEnabledModules(raw),
    ["imu"],
  );

  assertEquals(
    determineEnabledModules(raw, { includePilot: true }),
    ["imu", "pilot"],
  );
});

Deno.test(
  "determineEnabledModules falls back to configured launches when host is silent",
  () => {
    const raw = {
      modules: {
        ear: { launch: true },
        foot: { launch: false },
        memory: { launch: { enabled: true } },
      },
    } as Record<string, unknown>;

    assertEquals(
      determineEnabledModules(raw),
      ["ear", "memory"],
    );
  },
);

Deno.test(
  "determineEnabledModules treats declared modules without overrides as enabled",
  () => {
    const raw = {
      host: { modules: ["pilot", "nav"] },
      modules: {
        pilot: {},
        nav: {},
        ear: { launch: true },
      },
    } as Record<string, unknown>;

    assertEquals(
      determineEnabledModules(raw),
      ["nav"],
    );

    assertEquals(
      determineEnabledModules(raw, { includePilot: true }),
      ["nav", "pilot"],
    );
  },
);

Deno.test("determineEnabledServices reflects host declarations", () => {
  const raw = {
    host: { services: ["asr", "tts"] },
    services: {
      asr: {},
      graphs: { enabled: true },
      vectors: { enabled: false },
    },
  } as Record<string, unknown>;

  assertEquals(
    determineEnabledServices(raw),
    ["asr", "tts"],
  );
});

Deno.test(
  "determineEnabledServices falls back to configured services when host is silent",
  () => {
    const raw = {
      services: {
        asr: { enabled: true },
        graphs: { enabled: false },
        vectors: { enabled: "yes" },
      },
    } as Record<string, unknown>;

    assertEquals(
      determineEnabledServices(raw),
      ["asr", "vectors"],
    );
  },
);
