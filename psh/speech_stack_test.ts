import { assertEquals, assertRejects } from "@std/assert";
import type { SpeechService, SpeechWebSocket } from "./speech_stack.ts";
import {
  launchSpeechStack,
  stopSpeechStack,
  testSpeechStack,
} from "./speech_stack.ts";
import { createDaxStub, type StubInvocation } from "./test_utils.ts";

class StubSocket implements SpeechWebSocket {
  public readonly sent: Array<string | Uint8Array> = [];
  public closed = false;
  private readonly responses: Array<{ type: "text"; text: string } | { type: "binary"; bytes: Uint8Array }>;

  constructor(responses: Array<{ type: "text"; text: string } | { type: "binary"; bytes: Uint8Array }>) {
    this.responses = [...responses];
  }

  async send(data: string | Uint8Array): Promise<void> {
    this.sent.push(typeof data === "string" ? data : new Uint8Array(data));
  }

  async nextMessage(_timeoutMs: number): Promise<{ type: "text"; text: string } | { type: "binary"; bytes: Uint8Array }> {
    const response = this.responses.shift();
    if (!response) {
      throw new Error("no more responses queued");
    }
    return response;
  }

  async close(): Promise<void> {
    this.closed = true;
  }
}

function decodeSentJSON(messages: Array<string | Uint8Array>): unknown[] {
  return messages.map((entry) => {
    if (typeof entry === "string") {
      return JSON.parse(entry);
    }
    return Array.from(entry);
  });
}

Deno.test("launchSpeechStack runs docker compose up", async () => {
  let invocation: StubInvocation | null = null;
  const composeStub = createDaxStub((call) => {
    invocation = call;
    return { code: 0 };
  });

  await launchSpeechStack({ compose: composeStub, build: true });

  if (invocation === null) {
    throw new Error("expected docker compose to be invoked");
  }
  const { parts, values } = invocation;
  assertEquals(parts[0], "docker compose -f ");
  assertEquals(values[0], "speech-stack.compose.yml");
  assertEquals(parts[1], " ");
  assertEquals(values[1], "up -d --build");
  assertEquals(parts[2], "");
});

Deno.test("stopSpeechStack runs docker compose down", async () => {
  let invocation: StubInvocation | null = null;
  const composeStub = createDaxStub((call) => {
    invocation = call;
    return { code: 0 };
  });

  await stopSpeechStack({ compose: composeStub, volumes: true });

  if (invocation === null) {
    throw new Error("expected docker compose to be invoked");
  }
  const { parts, values } = invocation;
  assertEquals(parts[0], "docker compose -f ");
  assertEquals(values[0], "speech-stack.compose.yml");
  assertEquals(parts[1], " ");
  assertEquals(values[1], "down --volumes");
  assertEquals(parts[2], "");
});

Deno.test("testSpeechStack orchestrates compose and websocket checks", async () => {
  const composeCalls: StubInvocation[] = [];
  const composeStub = createDaxStub((call) => {
    composeCalls.push(call);
    return { code: 0 };
  });

  const sockets: Record<SpeechService, StubSocket> = {
    llm: new StubSocket([
      { type: "text", text: JSON.stringify({ role: "system", content: { generated_tokens: 0, uptime_ms: 1 } }) },
    ]),
    tts: new StubSocket([
      { type: "text", text: JSON.stringify({ event: "start", sample_rate: 16000, channels: 1 }) },
      { type: "binary", bytes: new Uint8Array([1, 2, 3, 4]) },
      { type: "text", text: JSON.stringify({ event: "end", num_samples: 4, duration_s: 0.00025 }) },
    ]),
    asr: new StubSocket([
      { type: "text", text: JSON.stringify({ type: "partial", stream_id: "psh-test", text: "samples=4 sum=6" }) },
      { type: "text", text: JSON.stringify({ type: "final", stream_id: "psh-test", text: "samples=4 sum=6", chunk_id: "chunk-1" }) },
    ]),
  };

  const connect = async (service: SpeechService): Promise<SpeechWebSocket> => {
    const socket = sockets[service];
    if (!socket) {
      throw new Error(`unexpected service ${service}`);
    }
    return socket;
  };

  await testSpeechStack({
    compose: composeStub,
    connect,
    handshakeTimeoutMs: 10,
    messageTimeoutMs: 10,
  });

  assertEquals(composeCalls.length, 1);
  const [call] = composeCalls;
  assertEquals(call.parts[0], "docker compose -f ");
  assertEquals(call.values[0], "speech-stack.compose.yml");
  assertEquals(call.parts[1], " ");
  assertEquals(call.values[1], "up -d");
  assertEquals(call.parts[2], "");

  const llmMessages = decodeSentJSON(sockets.llm.sent);
  assertEquals(llmMessages, [{ command: "stats" }]);
  const ttsMessages = decodeSentJSON(sockets.tts.sent);
  assertEquals(ttsMessages, [{ text: "Testing text-to-speech pipeline" }]);
  const asrMessages = decodeSentJSON(sockets.asr.sent);
  assertEquals(asrMessages.length, 3);
  assertEquals((asrMessages[0] as Record<string, unknown>).type, "init");
  assertEquals((asrMessages[1] as Record<string, unknown>).type, "audio");
  assertEquals((asrMessages[2] as Record<string, unknown>).type, "commit");

  assertEquals(sockets.llm.closed, true);
  assertEquals(sockets.tts.closed, true);
  assertEquals(sockets.asr.closed, true);
});

Deno.test("testSpeechStack surfaces compose failures", async () => {
  const composeStub = createDaxStub(() => ({ code: 1, stderr: "boom" }));
  await assertRejects(
    () =>
      testSpeechStack({
        compose: composeStub,
        connect: async () => new StubSocket([]),
      }),
    Error,
    "docker compose up",
  );
});
