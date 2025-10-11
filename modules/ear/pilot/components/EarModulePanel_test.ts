import { assertEquals } from "$std/assert/mod.ts";

import { __test__ } from "./EarModulePanel.tsx";

const { appendTranscriptLog } = __test__;

Deno.test("appendTranscriptLog keeps newest transcript first", () => {
  let log = appendTranscriptLog([], "hello world", {
    now: () => 1_000,
    limit: 3,
  });
  assertEquals(log, [
    { text: "hello world", receivedAt: 1_000 },
  ]);

  log = appendTranscriptLog(log, " second message ", {
    now: () => 2_000,
    limit: 3,
  });
  assertEquals(log, [
    { text: "second message", receivedAt: 2_000 },
    { text: "hello world", receivedAt: 1_000 },
  ]);
});

Deno.test("appendTranscriptLog enforces length and ignores duplicates", () => {
  const options = { limit: 2, now: () => 0 } as const;
  let log = appendTranscriptLog([], "first", { ...options, now: () => 100 });
  log = appendTranscriptLog(log, "second", { ...options, now: () => 200 });
  log = appendTranscriptLog(log, "second", { ...options, now: () => 300 });
  assertEquals(log, [
    { text: "second", receivedAt: 200 },
    { text: "first", receivedAt: 100 },
  ]);

  log = appendTranscriptLog(log, "third", { ...options, now: () => 400 });
  assertEquals(log, [
    { text: "third", receivedAt: 400 },
    { text: "second", receivedAt: 200 },
  ]);

  log = appendTranscriptLog(log, "   ", { ...options, now: () => 500 });
  assertEquals(log, [
    { text: "third", receivedAt: 400 },
    { text: "second", receivedAt: 200 },
  ]);
});
