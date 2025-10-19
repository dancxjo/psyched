import { strict as assert } from "node:assert";
import test from "node:test";

import {
  loadCollapsedCards,
  normaliseDebugSnapshot,
  normaliseFeelingIntent,
  parseRosStamp,
  persistCollapsedCards,
} from "./pilot-dashboard.helpers.js";

test("parseRosStamp returns Date for valid stamp", () => {
  const result = parseRosStamp({ sec: 1700000000, nanosec: 500000000 });
  assert.ok(result instanceof Date);
  assert.equal(result.getTime(), 1700000000_500);
});

test("parseRosStamp returns null for invalid input", () => {
  assert.equal(parseRosStamp({}), null);
  assert.equal(parseRosStamp(null), null);
});

test("normaliseFeelingIntent trims and preserves key fields", () => {
  const message = {
    stamp: { sec: 1700000100, nanosec: 0 },
    attitude_emoji: " 😀 ",
    spoken_sentence: " Hello there ",
    thought_sentence: " Considering actions ",
    command_script: ' nav.move_to(target="visitor") ',
    goals: ["  greet  ", 42, "", null],
    mood_delta: " +calm ",
    source_topics: [" /status ", ""],
    memory_collection_emoji: " 📚 ",
    memory_collection_text: " Archive updated ",
    memory_collection_raw: "",
    episode_id: " episode-1 ",
    situation_id: "",
  };
  const result = normaliseFeelingIntent(message);
  assert.ok(result);
  assert.equal(result.id, "episode-1");
  assert.equal(result.attitudeEmoji, "😀");
  assert.equal(result.spokenSentence, "Hello there");
  assert.equal(result.commandScript, 'nav.move_to(target="visitor")');
  assert.deepEqual(result.goals, ["greet"]);
  assert.deepEqual(result.sourceTopics, ["/status"]);
  assert.ok(result.stamp instanceof Date);
});

test("normaliseDebugSnapshot surfaces config and telemetry", () => {
  const snapshot = {
    status: " running ",
    heartbeat: "2024-07-18T12:00:00Z",
    config: {
      debounce_seconds: 2.5,
      window_seconds: "4",
      context_topics: ["/status", "/instant", "/status"],
      sensation_topics: ["/sensations"],
    },
    recent_sensations: [
      {
        topic: " /sensations ",
        kind: "touch",
        collection_hint: "hand",
        json_payload: '{"force": 1}',
        vector_len: 4,
      },
      { topic: "", kind: "ignored" },
    ],
    scripts: [
      {
        id: " run-1 ",
        status: " completed ",
        started_at: "2024-07-18T11:59:55Z",
        finished_at: "2024-07-18T12:00:01Z",
        used_actions: [" nav.move_to ", ""],
        actions: [{
          action: " nav.move_to ",
          status: " ok ",
          response: { ok: true },
          timestamp: "2024-07-18T12:00:00Z",
        }],
      },
    ],
    last_llm: " Hello world ",
    logs: [" entry ", "entry "],
    errors: [""],
  };

  const result = normaliseDebugSnapshot(snapshot);
  assert.equal(result.status, "running");
  assert.ok(result.heartbeat instanceof Date);
  assert.equal(result.config.debounceSeconds, 2.5);
  assert.equal(result.config.windowSeconds, 4);
  assert.deepEqual(result.config.contextTopics, ["/status", "/instant"]);
  assert.equal(result.recentSensations.length, 1);
  assert.equal(result.recentSensations[0].topic, "/sensations");
  assert.equal(result.scripts[0].status, "completed");
  assert.deepEqual(result.scripts[0].usedActions, ["nav.move_to"]);
  assert.equal(result.lastLLM, "Hello world");
  assert.equal(result.logs.length, 1);
  assert.equal(result.errors.length, 0);
});

test("loadCollapsedCards returns stored identifiers", () => {
  const storage = {
    getItem(key) {
      if (key === "psyched::pilot-dashboard::collapsed") {
        return JSON.stringify([" pilot-status ", "pilot-scripts", ""]);
      }
      return null;
    },
  };

  const collapsed = loadCollapsedCards(storage);
  assert.ok(collapsed instanceof Set);
  assert.deepEqual(Array.from(collapsed), ["pilot-status", "pilot-scripts"]);
});

test("loadCollapsedCards copes with corrupt storage entries", () => {
  const storage = {
    getItem() {
      return "not-json";
    },
  };

  const collapsed = loadCollapsedCards(storage);
  assert.ok(collapsed instanceof Set);
  assert.equal(collapsed.size, 0);
});

test("persistCollapsedCards stores unique trimmed identifiers", () => {
  const writes = [];
  const storage = {
    setItem(key, value) {
      writes.push([key, value]);
    },
  };

  const source = new Set([" pilot-status ", "pilot-status", "pilot-logs"]);
  const stored = persistCollapsedCards(source, storage);

  assert.equal(stored, true);
  assert.deepEqual(writes, [[
    "psyched::pilot-dashboard::collapsed",
    JSON.stringify(["pilot-status", "pilot-logs"]),
  ]]);
});

test("persistCollapsedCards tolerates storage failures", () => {
  const storage = {
    setItem() {
      throw new Error("write failed");
    },
  };

  const stored = persistCollapsedCards(new Set(["pilot-status"]), storage);
  assert.equal(stored, false);
});
