import { createContext, createElement } from "preact";
import type { ComponentChildren } from "preact";
import { useContext, useEffect, useMemo, useRef, useState } from "preact/hooks";
import {
  defaultCockpitUrl,
  isBrowser,
  readBootstrappedConfig,
} from "./cockpit_url.ts";
import {
  clearCockpitConnectionError,
  recordCockpitConnectionError,
  updateCockpitConnectionStatus,
} from "./cockpit_signals.ts";

export type ConnectionStatus =
  | "idle"
  | "connecting"
  | "open"
  | "closed"
  | "error";

export interface CockpitClientOptions {
  /** Override the websocket endpoint. Defaults to ws(s)://host:8088/ws */
  url?: string;
  /** Automatically reconnect when the connection drops. Defaults to true. */
  autoReconnect?: boolean;
  /** Milliseconds to wait before attempting reconnection. Defaults to 2000. */
  reconnectDelayMs?: number;
}

type TopicCallback<T = unknown> = (payload: T) => void;

interface SubscriptionRecord {
  callbacks: Set<TopicCallback>;
  /** Last payload received for this topic. */
  lastPayload?: unknown;
  /** True when a subscription request still has to be sent once connected. */
  pending: boolean;
}

interface WsOutboundMessage {
  op: "msg" | "ok" | "err";
  topic?: string;
  msg?: unknown;
  reason?: string;
}

interface SubscribeOptions {
  /** Replay the last payload right away if available. Defaults to true. */
  replay?: boolean;
}

const DEFAULT_RECONNECT_DELAY_MS = 2000;

type ResolvedCockpitClientOptions = {
  url?: string;
  autoReconnect: boolean;
  reconnectDelayMs: number;
};

const CockpitClientContext = createContext<CockpitClient | null>(null);
const CockpitStatusContext = createContext<ConnectionStatus>("idle");

function resolveProviderOptions(
  options?: CockpitClientOptions,
): ResolvedCockpitClientOptions {
  return {
    url: options?.url,
    autoReconnect: options?.autoReconnect ?? true,
    reconnectDelayMs: options?.reconnectDelayMs ?? DEFAULT_RECONNECT_DELAY_MS,
  };
}

function serializeOptions(options: ResolvedCockpitClientOptions): string {
  return [
    options.url ?? "",
    options.autoReconnect ? "1" : "0",
    String(options.reconnectDelayMs),
  ].join("|");
}

export interface CockpitProviderProps {
  children: ComponentChildren;
  options?: CockpitClientOptions;
  autoConnect?: boolean;
}

export function CockpitProvider({
  children,
  options,
  autoConnect = true,
}: CockpitProviderProps) {
  const resolvedOptions = useMemo(
    () => resolveProviderOptions(options),
    [options?.url, options?.autoReconnect, options?.reconnectDelayMs],
  );
  const optionsSignature = useMemo(
    () => serializeOptions(resolvedOptions),
    [resolvedOptions],
  );
  const signatureRef = useRef(optionsSignature);
  const [client, setClient] = useState<CockpitClient>(() =>
    createCockpitClient(resolvedOptions)
  );
  const [status, setStatus] = useState<ConnectionStatus>(
    client.connectionStatus,
  );

  useEffect(() => {
    if (signatureRef.current === optionsSignature) {
      return;
    }
    signatureRef.current = optionsSignature;
    setClient((previous) => {
      previous?.disconnect();
      return createCockpitClient(resolvedOptions);
    });
  }, [optionsSignature, resolvedOptions]);

  useEffect(() => {
    setStatus(client.connectionStatus);
    return client.onStatusChange((next) => {
      setStatus(next);
    });
  }, [client]);

  useEffect(() => {
    if (!autoConnect) {
      return;
    }
    client.connect();
    return () => {
      client.disconnect();
    };
  }, [client, autoConnect]);

  return createElement(
    CockpitClientContext.Provider,
    { value: client },
    createElement(
      CockpitStatusContext.Provider,
      { value: status },
      children,
    ),
  );
}

export function useCockpitClient(): CockpitClient {
  const client = useContext(CockpitClientContext);
  return client ?? getDefaultCockpitClient();
}

export function useCockpitConnectionStatus(): ConnectionStatus {
  return useContext(CockpitStatusContext);
}

export class CockpitClient {
  private socket: WebSocket | null = null;
  private status: ConnectionStatus = "idle";
  private readonly subscriptions = new Map<string, SubscriptionRecord>();
  private readonly statusListeners = new Set<
    (status: ConnectionStatus) => void
  >();
  private readonly options: Required<CockpitClientOptions>;
  private reconnectTimer: number | null = null;
  private manualDisconnect = false;

  constructor(opts: CockpitClientOptions = {}) {
    this.options = {
      url: opts.url ?? defaultCockpitUrl(),
      autoReconnect: opts.autoReconnect ?? true,
      reconnectDelayMs: opts.reconnectDelayMs ?? DEFAULT_RECONNECT_DELAY_MS,
    };

    updateCockpitConnectionStatus(this.status);

    if (isBrowser && this.options.autoReconnect) {
      // Eagerly connect so hooks receive status updates.
      queueMicrotask(() => this.connect());
    }
  }

  get connectionStatus(): ConnectionStatus {
    return this.status;
  }

  connect(): void {
    if (!isBrowser) {
      return;
    }

    if (
      this.socket &&
      (this.socket.readyState === WebSocket.CONNECTING ||
        this.socket.readyState === WebSocket.OPEN)
    ) {
      return;
    }

    if (!this.options.url) {
      console.warn("CockpitClient cannot connect without a URL.");
      recordCockpitConnectionError("missing cockpit websocket URL");
      this.setStatus("error");
      return;
    }

    this.clearReconnectTimer();

    this.manualDisconnect = false;
    this.setStatus("connecting");
    console.debug(
      "CockpitClient connecting to",
      this.options.url,
    );
    try {
      this.socket = new WebSocket(this.options.url);
    } catch (error) {
      console.error("Failed to open cockpit websocket", error);
      this.setStatus("error");
      recordCockpitConnectionError("failed to open cockpit websocket");
      this.scheduleReconnect();
      return;
    }

    this.socket.addEventListener("open", () => {
      this.setStatus("open");
      clearCockpitConnectionError();
      // Resubscribe to all topics when the connection is re-established.
      for (const [topic, record] of this.subscriptions.entries()) {
        record.pending = true;
        this.sendSubscribe(topic);
      }
    });

    this.socket.addEventListener("close", (event) => {
      console.debug(
        "Cockpit websocket closed",
        { code: event.code, wasClean: event.wasClean },
      );
      this.setStatus("closed");
      if (this.options.autoReconnect && !this.manualDisconnect) {
        this.scheduleReconnect();
      }
    });

    this.socket.addEventListener("error", (event) => {
      console.error("Cockpit websocket error", event);
      this.setStatus("error");
      recordCockpitConnectionError("cockpit websocket error");
    });

    this.socket.addEventListener("message", (event) => {
      console.debug("Cockpit websocket message", { type: typeof event.data });
      void this.handleInbound(event.data);
    });
  }

  disconnect(): void {
    this.manualDisconnect = true;
    this.clearReconnectTimer();
    if (this.socket) {
      this.socket.close();
      this.socket = null;
      this.setStatus("closed");
    }
  }

  publish<T>(topic: string, msg: T): void {
    this.ensureSocket();
    if (!this.socket || this.socket.readyState !== WebSocket.OPEN) {
      return;
    }
    const payload = JSON.stringify({ op: "pub", topic, msg });
    this.socket.send(payload);
  }

  subscribe<T>(
    topic: string,
    callback: TopicCallback<T>,
    options: SubscribeOptions = {},
  ): () => void {
    if (!isBrowser) {
      return () => undefined;
    }

    const record = this.getOrCreateRecord(topic);
    record.callbacks.add(callback as TopicCallback);

    if (options.replay !== false && record.lastPayload !== undefined) {
      const invoke = () => callback(record.lastPayload as T);
      if (typeof queueMicrotask === "function") {
        queueMicrotask(invoke);
      } else {
        Promise.resolve().then(invoke);
      }
    }

    this.ensureSocket();
    if (this.socket?.readyState === WebSocket.OPEN) {
      this.sendSubscribe(topic);
    } else {
      record.pending = true;
    }

    return () => {
      const existing = this.subscriptions.get(topic);
      if (!existing) return;
      existing.callbacks.delete(callback as TopicCallback);
      if (existing.callbacks.size === 0) {
        this.subscriptions.delete(topic);
        if (this.socket?.readyState === WebSocket.OPEN) {
          this.sendUnsubscribe(topic);
        }
      }
    };
  }

  onStatusChange(listener: (status: ConnectionStatus) => void): () => void {
    this.statusListeners.add(listener);
    listener(this.status);
    return () => {
      this.statusListeners.delete(listener);
    };
  }

  private ensureSocket(): void {
    if (!isBrowser) {
      return;
    }
    if (!this.socket || this.socket.readyState === WebSocket.CLOSED) {
      this.connect();
    }
  }

  private async handleInbound(raw: unknown): Promise<void> {
    let payloadRaw = raw;
    if (typeof Blob !== "undefined" && raw instanceof Blob) {
      payloadRaw = await raw.text();
    } else if (
      typeof ArrayBuffer !== "undefined" && raw instanceof ArrayBuffer
    ) {
      payloadRaw = new TextDecoder().decode(raw);
    }

    let payload: WsOutboundMessage;
    try {
      payload = typeof payloadRaw === "string"
        ? JSON.parse(payloadRaw)
        : payloadRaw as WsOutboundMessage;
    } catch (error) {
      console.warn("Failed to parse cockpit message", error, raw);
      recordCockpitConnectionError("failed to parse cockpit message");
      return;
    }

    if (!payload || typeof payload !== "object" || !("op" in payload)) {
      return;
    }

    switch (payload.op) {
      case "msg": {
        if (!payload.topic) return;
        const record = this.subscriptions.get(payload.topic);
        if (!record) return;
        record.lastPayload = payload.msg;
        for (const cb of record.callbacks) {
          try {
            cb(payload.msg);
          } catch (error) {
            console.error("Cockpit subscriber threw", error);
          }
        }
        break;
      }
      case "err": {
        console.error(
          "Cockpit reported error",
          payload.reason ?? "unknown error",
        );
        recordCockpitConnectionError(
          payload.reason ?? "unknown cockpit error",
        );
        break;
      }
      case "ok":
      default:
        break;
    }
  }

  private sendSubscribe(topic: string): void {
    const record = this.subscriptions.get(topic);
    if (!record) return;
    if (!this.socket || this.socket.readyState !== WebSocket.OPEN) {
      record.pending = true;
      return;
    }
    if (!record.pending) {
      // We already subscribed earlier in this session and no reconnection happened.
      return;
    }
    this.socket.send(JSON.stringify({ op: "sub", topic }));
    record.pending = false;
  }

  private sendUnsubscribe(topic: string): void {
    if (!this.socket || this.socket.readyState !== WebSocket.OPEN) {
      return;
    }
    this.socket.send(JSON.stringify({ op: "unsub", topic }));
  }

  private getOrCreateRecord(topic: string): SubscriptionRecord {
    let record = this.subscriptions.get(topic);
    if (!record) {
      record = { callbacks: new Set(), pending: true };
      this.subscriptions.set(topic, record);
    }
    return record;
  }

  private setStatus(status: ConnectionStatus): void {
    if (this.status === status) return;
    this.status = status;
    updateCockpitConnectionStatus(status);
    for (const listener of this.statusListeners) {
      try {
        listener(status);
      } catch (error) {
        console.error("Cockpit status listener threw", error);
      }
    }
  }

  private scheduleReconnect(): void {
    if (!this.options.autoReconnect || !isBrowser || this.manualDisconnect) {
      return;
    }
    this.clearReconnectTimer();
    console.debug(
      "CockpitClient scheduling reconnect in",
      this.options.reconnectDelayMs,
      "ms",
    );
    this.reconnectTimer = globalThis.setTimeout(() => {
      this.reconnectTimer = null;
      this.connect();
    }, this.options.reconnectDelayMs);
  }

  private clearReconnectTimer(): void {
    if (this.reconnectTimer !== null && isBrowser) {
      globalThis.clearTimeout(this.reconnectTimer);
      this.reconnectTimer = null;
    }
  }
}

export function createCockpitClient(
  options?: CockpitClientOptions,
): CockpitClient {
  return new CockpitClient(options);
}

let defaultClient: CockpitClient | null = null;

export function getDefaultCockpitClient(): CockpitClient {
  if (!defaultClient) {
    defaultClient = new CockpitClient();
  }
  return defaultClient;
}

interface UseTopicOptions<T> {
  initialValue?: T;
  /** Disable automatic connection attempts when subscribing. */
  autoConnect?: boolean;
  /** Replay buffered payload (default true). */
  replay?: boolean;
}

export function useCockpitTopic<T = unknown>(
  topic: string,
  options: UseTopicOptions<T> = {},
  explicitClient?: CockpitClient,
) {
  const contextClient = useContext(CockpitClientContext);
  const client = explicitClient ?? contextClient ?? getDefaultCockpitClient();
  const { initialValue, autoConnect = true, replay = true } = options;
  const [payload, setPayload] = useState<T | undefined>(initialValue);
  const [status, setStatus] = useState<ConnectionStatus>(
    client.connectionStatus,
  );
  const [error, setError] = useState<string | null>(null);
  const topicRef = useRef(topic);

  // Keep latest handler stable for cleanup.
  const handlerRef = useRef<(value: T) => void>();
  handlerRef.current = (value: T) => {
    setPayload(value);
  };

  useEffect(() => {
    if (!isBrowser) {
      return;
    }

    if (autoConnect) {
      client.connect();
    }

    const unsubscribe = client.subscribe<T>(topic, (value) => {
      handlerRef.current?.(value);
      setError(null);
    }, { replay });

    return () => {
      unsubscribe();
    };
  }, [topic, client, autoConnect, replay]);

  useEffect(() => {
    topicRef.current = topic;
  }, [topic]);

  useEffect(() => {
    return client.onStatusChange((next) => {
      setStatus(next);
      if (next === "error") {
        setError("cockpit connection error");
      }
    });
  }, [client]);

  return useMemo(
    () => ({
      topic,
      data: payload,
      status,
      error,
      publish: (value: T) => client.publish(topicRef.current, value),
    }),
    [topic, payload, status, error, client],
  );
}

export const __test__ = {
  defaultCockpitUrl,
  readBootstrappedConfig,
};
