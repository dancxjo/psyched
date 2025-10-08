interface CockpitBootstrapConfig {
  host?: string;
  port?: string;
  protocol?: string;
  url?: string;
}

export const globalWindow = typeof globalThis === "object" &&
    "window" in globalThis
  ? (globalThis as typeof globalThis & { window: Window }).window
  : undefined;

export const isBrowser = Boolean(globalWindow?.WebSocket);

export function readBootstrappedConfig(): CockpitBootstrapConfig | undefined {
  if (!isBrowser || !globalWindow?.document?.body?.dataset) {
    return undefined;
  }
  const dataset = globalWindow.document.body.dataset;
  const sanitize = (value: string | undefined): string | undefined => {
    const trimmed = value?.trim();
    return trimmed ? trimmed : undefined;
  };
  const config: CockpitBootstrapConfig = {
    host: sanitize(dataset.cockpitHost),
    port: sanitize(dataset.cockpitPort),
    protocol: sanitize(dataset.cockpitProtocol),
    url: sanitize(dataset.cockpitUrl),
  };
  return Object.values(config).some((value) => value !== undefined)
    ? config
    : undefined;
}

export function defaultCockpitUrl(): string {
  if (!isBrowser || !globalWindow) {
    return "";
  }
  const { protocol, hostname, port } = globalWindow.location;
  const bootstrap = readBootstrappedConfig();
  if (bootstrap?.url) {
    return bootstrap.url;
  }
  const normalisedProtocol = (() => {
    const raw = bootstrap?.protocol?.toLowerCase();
    if (!raw) return protocol === "https:" ? "wss" : "ws";
    if (raw === "ws" || raw === "wss") return raw;
    if (raw === "https" || raw === "https:") return "wss";
    if (raw === "http" || raw === "http:") return "ws";
    return raw;
  })();
  const inferredPort = (() => {
    if (bootstrap?.port) return bootstrap.port;
    if (port === "" || port === undefined) return "8088";
    if (port === "8000") return "8088";
    return port;
  })();
  const targetHost = bootstrap?.host ?? hostname;
  const portSegment = inferredPort ? `:${inferredPort}` : "";
  return `${normalisedProtocol}://${targetHost}${portSegment}/ws`;
}

export const __test__ = {
  defaultCockpitUrl,
  readBootstrappedConfig,
};
