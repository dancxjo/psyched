import { basename, dirname } from "@std/path";
import { encodeBase64 } from "@std/encoding/base64";
import { $, type DaxTemplateTag } from "./util.ts";
import { repoPath } from "./util.ts";

export type SpeechService = "llm" | "tts" | "asr";

export interface SpeechWebSocket {
  send(data: string | Uint8Array): Promise<void>;
  nextMessage(timeoutMs: number): Promise<{ type: "text"; text: string } | { type: "binary"; bytes: Uint8Array }>;
  close(code?: number): Promise<void>;
}

export interface SpeechStackLaunchOptions {
  build?: boolean;
  compose?: DaxTemplateTag;
}

export interface SpeechStackStopOptions {
  volumes?: boolean;
  compose?: DaxTemplateTag;
}

export interface SpeechStackTestOptions extends SpeechStackLaunchOptions {
  connect?: (service: SpeechService, url: string) => Promise<SpeechWebSocket>;
  handshakeTimeoutMs?: number;
  messageTimeoutMs?: number;
}

const COMPOSE_PATH = repoPath("../compose/speech-stack.compose.yml");
const COMPOSE_DIR = dirname(COMPOSE_PATH);
const COMPOSE_FILE = basename(COMPOSE_PATH);

async function runCompose(
  args: string[],
  { compose }: { compose?: DaxTemplateTag },
): Promise<void> {
  const runner = compose ?? $;
  const command = runner`docker compose -f ${COMPOSE_FILE} ${args.join(" ")}`
    .cwd(COMPOSE_DIR)
    .stdout("inherit")
    .stderr("inherit");
  const result = await command.noThrow();
  const code = result.code ?? 0;
  if (code !== 0) {
    throw new Error(`[psh] docker compose ${args[0]} failed (exit ${code}).`);
  }
}

/**
 * Launch the speech stack docker compose services in detached mode.
 *
 * @example
 * ```ts
 * await launchSpeechStack();
 * await launchSpeechStack({ build: true });
 * ```
 */
export async function launchSpeechStack(options: SpeechStackLaunchOptions = {}): Promise<void> {
  const args = ["up", "-d"];
  if (options.build) {
    args.push("--build");
  }
  console.log(`[psh] Starting speech stack via docker compose (${args.join(" ")})...`);
  await runCompose(args, options);
  console.log("[psh] Speech stack containers are running.");
}

/**
 * Stop the speech stack containers started via docker compose.
 *
 * @example
 * ```ts
 * await stopSpeechStack();
 * await stopSpeechStack({ volumes: true });
 * ```
 */
export async function stopSpeechStack(options: SpeechStackStopOptions = {}): Promise<void> {
  const args = ["down"];
  if (options.volumes) {
    args.push("--volumes");
  }
  console.log(`[psh] Stopping speech stack via docker compose (${args.join(" ")})...`);
  await runCompose(args, options);
  console.log("[psh] Speech stack containers stopped.");
}

class BrowserSpeechSocket implements SpeechWebSocket {
  constructor(private readonly socket: WebSocket) {}

  async send(data: string | Uint8Array): Promise<void> {
    if (typeof data === "string") {
      this.socket.send(data);
    } else {
      this.socket.send(data);
    }
  }

  async nextMessage(
    timeoutMs: number,
  ): Promise<{ type: "text"; text: string } | { type: "binary"; bytes: Uint8Array }> {
    return await new Promise((resolve, reject) => {
      const timeout = setTimeout(() => {
        cleanup();
        reject(new Error(`[psh] Timed out waiting for websocket message from ${this.socket.url}`));
      }, timeoutMs);

      const handleMessage = (event: MessageEvent) => {
        cleanup();
        const payload = event.data;
        if (typeof payload === "string") {
          resolve({ type: "text", text: payload });
          return;
        }
        if (payload instanceof ArrayBuffer) {
          resolve({ type: "binary", bytes: new Uint8Array(payload) });
          return;
        }
        if (payload instanceof Uint8Array) {
          resolve({ type: "binary", bytes: payload });
          return;
        }
        if (payload instanceof Blob) {
          payload.arrayBuffer()
            .then((buffer) => resolve({ type: "binary", bytes: new Uint8Array(buffer) }))
            .catch((err) => reject(err));
          return;
        }
        reject(new Error("[psh] Received unsupported websocket payload type."));
      };

      const handleError = (event: Event | ErrorEvent) => {
        cleanup();
        if (event instanceof ErrorEvent && event.error) {
          reject(event.error);
        } else {
          reject(new Error(`[psh] WebSocket error from ${this.socket.url}`));
        }
      };

      const handleClose = () => {
        cleanup();
        reject(new Error(`[psh] WebSocket ${this.socket.url} closed before a message was received.`));
      };

      const cleanup = () => {
        clearTimeout(timeout);
        this.socket.removeEventListener("message", handleMessage);
        this.socket.removeEventListener("error", handleError);
        this.socket.removeEventListener("close", handleClose);
      };

      this.socket.addEventListener("message", handleMessage, { once: true });
      this.socket.addEventListener("error", handleError, { once: true });
      this.socket.addEventListener("close", handleClose, { once: true });
    });
  }

  async close(code = 1000): Promise<void> {
    if (this.socket.readyState === WebSocket.CLOSED) {
      return;
    }
    if (this.socket.readyState === WebSocket.CLOSING) {
      await new Promise<void>((resolve) => this.socket.addEventListener("close", () => resolve(), { once: true }));
      return;
    }
    await new Promise<void>((resolve) => {
      const handleClose = () => {
        this.socket.removeEventListener("close", handleClose);
        resolve();
      };
      this.socket.addEventListener("close", handleClose, { once: true });
      try {
        this.socket.close(code);
      } catch {
        resolve();
      }
    });
  }
}

async function connectRealWebSocket(url: string, timeoutMs: number): Promise<SpeechWebSocket> {
  return await new Promise((resolve, reject) => {
    const socket = new WebSocket(url);
    socket.binaryType = "arraybuffer";

    const timeout = setTimeout(() => {
      cleanup();
      try {
        socket.close();
      } catch {
        // ignore
      }
      reject(new Error(`[psh] Timed out connecting to ${url}`));
    }, timeoutMs);

    const handleOpen = () => {
      cleanup();
      resolve(new BrowserSpeechSocket(socket));
    };

    const handleError = (event: Event | ErrorEvent) => {
      cleanup();
      if (event instanceof ErrorEvent && event.error) {
        reject(event.error);
      } else {
        reject(new Error(`[psh] Failed to connect to ${url}`));
      }
    };

    const cleanup = () => {
      clearTimeout(timeout);
      socket.removeEventListener("open", handleOpen);
      socket.removeEventListener("error", handleError);
    };

    socket.addEventListener("open", handleOpen, { once: true });
    socket.addEventListener("error", handleError, { once: true });
  });
}

function encodePcm16(samples: readonly number[]): string {
  const bytes = new Uint8Array(samples.length * 2);
  samples.forEach((sample, index) => {
    const value = Math.max(-32768, Math.min(32767, Math.trunc(sample)));
    const offset = index * 2;
    bytes[offset] = value & 0xff;
    bytes[offset + 1] = (value >> 8) & 0xff;
  });
  return encodeBase64(bytes);
}

function parseJson<T>(text: string, context: string): T {
  try {
    return JSON.parse(text) as T;
  } catch (error) {
    throw new Error(`${context}: ${error instanceof Error ? error.message : String(error)}`);
  }
}

async function withServiceSocket(
  service: SpeechService,
  url: string,
  connect: (service: SpeechService, url: string) => Promise<SpeechWebSocket>,
  messageTimeoutMs: number,
  handler: (socket: SpeechWebSocket) => Promise<void>,
): Promise<void> {
  const socket = await connect(service, url);
  try {
    await handler(socket);
  } finally {
    await socket.close();
  }
}

/**
 * Launch (if necessary) and exercise the websocket interfaces exposed by the
 * speech stack services.
 *
 * @example
 * ```ts
 * await testSpeechStack();
 * await testSpeechStack({ build: true });
 * ```
 */
export async function testSpeechStack(options: SpeechStackTestOptions = {}): Promise<void> {
  const { build = false } = options;
  await launchSpeechStack({ build, compose: options.compose });

  const handshakeTimeoutMs = options.handshakeTimeoutMs ?? 5000;
  const messageTimeoutMs = options.messageTimeoutMs ?? 5000;
  const connect = options.connect ?? ((service: SpeechService, url: string) => connectRealWebSocket(url, handshakeTimeoutMs));

  console.log("[psh] Validating LLM websocket at ws://127.0.0.1:8080/chat ...");
  await withServiceSocket("llm", "ws://127.0.0.1:8080/chat", connect, messageTimeoutMs, async (socket) => {
    await socket.send(JSON.stringify({ command: "stats" }));
    const message = await socket.nextMessage(messageTimeoutMs);
    if (message.type !== "text") {
      throw new Error("[psh] LLM websocket returned non-text payload for stats request.");
    }
    const payload = parseJson<{ role?: string; content?: unknown }>(message.text, "[psh] Failed to parse LLM stats response");
    if (payload.role !== "system" || typeof payload.content !== "object" || payload.content === null) {
      throw new Error("[psh] LLM stats response did not include expected system payload.");
    }
  });
  console.log("[psh] LLM websocket responded with system stats.");

  console.log("[psh] Validating TTS websocket at ws://127.0.0.1:5002/tts ...");
  await withServiceSocket("tts", "ws://127.0.0.1:5002/tts", connect, messageTimeoutMs, async (socket) => {
    await socket.send(JSON.stringify({ text: "Testing text-to-speech pipeline" }));
    const start = await socket.nextMessage(messageTimeoutMs);
    if (start.type !== "text") {
      throw new Error("[psh] TTS websocket did not return JSON metadata.");
    }
    const metadata = parseJson<{ event?: string; sample_rate?: number; channels?: number }>(
      start.text,
      "[psh] Failed to parse TTS metadata",
    );
    if (metadata.event !== "start") {
      throw new Error("[psh] TTS websocket did not emit start metadata.");
    }
    const chunk = await socket.nextMessage(messageTimeoutMs);
    if (chunk.type !== "binary" || chunk.bytes.length === 0) {
      throw new Error("[psh] TTS websocket did not stream audio data.");
    }
    const end = await socket.nextMessage(messageTimeoutMs);
    if (end.type !== "text") {
      throw new Error("[psh] TTS websocket did not send end-of-stream metadata.");
    }
    const footer = parseJson<{ event?: string }>(end.text, "[psh] Failed to parse TTS completion message");
    if (footer.event !== "end") {
      throw new Error("[psh] TTS websocket did not emit end event.");
    }
  });
  console.log("[psh] TTS websocket produced audio stream and end marker.");

  console.log("[psh] Validating ASR websocket at ws://127.0.0.1:8082/ws ...");
  await withServiceSocket("asr", "ws://127.0.0.1:8082/ws", connect, messageTimeoutMs, async (socket) => {
    const streamId = "psh-test";
    await socket.send(
      JSON.stringify({
        type: "init",
        stream_id: streamId,
        lang: "en",
        content_type: "audio/pcm; rate=16000",
        sample_rate: 16_000,
      }),
    );
    const payload = encodePcm16([0, 1024, -1024, 2048]);
    await socket.send(
      JSON.stringify({
        type: "audio",
        stream_id: streamId,
        seq: 1,
        payload_b64: payload,
      }),
    );
    const partial = await socket.nextMessage(messageTimeoutMs);
    if (partial.type !== "text") {
      throw new Error("[psh] ASR websocket did not emit partial transcript.");
    }
    const partialPayload = parseJson<{ type?: string }>(partial.text, "[psh] Failed to parse ASR partial message");
    if (partialPayload.type !== "partial") {
      throw new Error("[psh] ASR websocket did not respond with a partial transcript.");
    }
    await socket.send(
      JSON.stringify({
        type: "commit",
        stream_id: streamId,
        chunk_id: "chunk-1",
      }),
    );
    const finalMessage = await socket.nextMessage(messageTimeoutMs);
    if (finalMessage.type !== "text") {
      throw new Error("[psh] ASR websocket did not send final transcript.");
    }
    const finalPayload = parseJson<{ type?: string }>(finalMessage.text, "[psh] Failed to parse ASR final message");
    if (finalPayload.type !== "final") {
      throw new Error("[psh] ASR websocket did not emit a final transcript.");
    }
  });
  console.log("[psh] ASR websocket produced partial and final transcripts.");

  console.log("[psh] Speech stack services responded successfully.");
}
