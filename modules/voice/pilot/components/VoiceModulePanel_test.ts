import { assertEquals } from "$std/assert/mod.ts";

import { __test__ } from "./VoiceModulePanel.tsx";

const { appendConversationLog } = __test__;

Deno.test("appendConversationLog records most recent messages first", () => {
  let log = appendConversationLog([], {
    text: "Hello",
    source: "remote",
  }, {
    now: () => 1_000,
    limit: 3,
  });
  assertEquals(log, [
    { text: "Hello", source: "remote", receivedAt: 1_000 },
  ]);

  log = appendConversationLog(log, {
    text: "Hi there",
    source: "local",
  }, {
    now: () => 2_000,
    limit: 3,
  });
  assertEquals(log, [
    { text: "Hi there", source: "local", receivedAt: 2_000 },
    { text: "Hello", source: "remote", receivedAt: 1_000 },
  ]);
});

Deno.test("appendConversationLog trims list and avoids duplicate echoes", () => {
  const options = { limit: 2 } as const;
  let log = appendConversationLog([], {
    text: "One",
    source: "local",
  }, { ...options, now: () => 100 });
  log = appendConversationLog(log, {
    text: "One",
    source: "remote",
  }, { ...options, now: () => 200 });
  assertEquals(log, [
    { text: "One", source: "local", receivedAt: 100 },
  ]);

  log = appendConversationLog(log, {
    text: "Two",
    source: "remote",
  }, { ...options, now: () => 300 });
  log = appendConversationLog(log, {
    text: "Three",
    source: "remote",
  }, { ...options, now: () => 400 });
  assertEquals(log, [
    { text: "Three", source: "remote", receivedAt: 400 },
    { text: "Two", source: "remote", receivedAt: 300 },
  ]);

  log = appendConversationLog(log, {
    text: "   ",
    source: "remote",
  }, { ...options, now: () => 500 });
  assertEquals(log, [
    { text: "Three", source: "remote", receivedAt: 400 },
    { text: "Two", source: "remote", receivedAt: 300 },
  ]);
});
