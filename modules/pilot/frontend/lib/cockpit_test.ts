/// <reference lib="dom" />

import { assertEquals } from "https://deno.land/std@0.216.0/testing/asserts.ts";

type BrowserOptions = {
  protocol?: string;
  hostname?: string;
  port?: string;
  dataset?: Record<string, string | undefined>;
};

async function withBrowserEnv(
  options: BrowserOptions,
  run: () => Promise<void>,
): Promise<void> {
  const previousWindow = (globalThis as { window?: Window }).window;
  const previousDocument = (globalThis as { document?: Document }).document;

  const dataset: Record<string, string> = {};
  for (const [key, value] of Object.entries(options.dataset ?? {})) {
    if (value) {
      dataset[key] = value;
    }
  }

  const body = {
    dataset: dataset as unknown as DOMStringMap,
  } as unknown as HTMLBodyElement;
  const document = { body } as unknown as Document;
  const windowStub = {
    location: {
      protocol: options.protocol ?? "http:",
      hostname: options.hostname ?? "localhost",
      port: options.port ?? "",
    },
    document,
    WebSocket: class WebSocketStub {} as unknown as typeof WebSocket,
  } as unknown as Window;

  (globalThis as { window?: Window }).window = windowStub;
  (globalThis as { document?: Document }).document = document;

  try {
    await run();
  } finally {
    if (previousWindow === undefined) {
      delete (globalThis as { window?: Window }).window;
    } else {
      (globalThis as { window?: Window }).window = previousWindow;
    }
    if (previousDocument === undefined) {
      delete (globalThis as { document?: Document }).document;
    } else {
      (globalThis as { document?: Document }).document = previousDocument;
    }
  }
}

async function importCockpitModule() {
  const cacheBust = crypto.randomUUID();
  return await import(`./cockpit_url.ts?cache=${cacheBust}`);
}

Deno.test("defaults to bridge port 8088 when dev server runs on 8000", async () => {
  await withBrowserEnv({ port: "8000" }, async () => {
    const { __test__ } = await importCockpitModule();
    const url = __test__.defaultCockpitUrl();
    assertEquals(url, "ws://localhost:8088/ws");
  });
});

Deno.test("uses bootstrapped cockpit port when provided", async () => {
  await withBrowserEnv({
    port: "8000",
    dataset: { cockpitPort: "9090" },
  }, async () => {
    const { __test__ } = await importCockpitModule();
    const url = __test__.defaultCockpitUrl();
    assertEquals(url, "ws://localhost:9090/ws");
  });
});

Deno.test("prefers explicit cockpit URL overrides", async () => {
  await withBrowserEnv({
    port: "8000",
    dataset: { cockpitUrl: "wss://pete.local:9443/bridge" },
  }, async () => {
    const { __test__ } = await importCockpitModule();
    const url = __test__.defaultCockpitUrl();
    assertEquals(url, "wss://pete.local:9443/bridge");
  });
});
