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

const DEV_SERVER_PORTS = new Set(["8000", "5173"]);

function normaliseProtocol(value: string | undefined): string | undefined {
  if (!value) return undefined;
  const normalised = value.toLowerCase().replace(/:$/, "");
  if (normalised === "ws" || normalised === "wss") return normalised;
  if (normalised === "https") return "wss";
  if (normalised === "http") return "ws";
  return normalised;
}

function tryParseUrl(value: string): URL | undefined {
  const trimmed = value.trim();
  if (!trimmed) return undefined;

  const attempts = new Set<string>();
  const hasScheme = /^[a-zA-Z][a-zA-Z0-9+.-]*:/.test(trimmed);
  if (hasScheme) {
    attempts.add(trimmed);
  } else {
    attempts.add(`http://${trimmed}`);
    attempts.add(`ws://${trimmed}`);
  }

  const colonCount = (trimmed.match(/:/g) ?? []).length;
  if (!trimmed.includes("[") && colonCount > 1) {
    attempts.add(`http://[${trimmed}]`);
    attempts.add(`ws://[${trimmed}]`);

    const lastColon = trimmed.lastIndexOf(":");
    if (lastColon !== -1) {
      const hostPart = trimmed.slice(0, lastColon);
      const portPart = trimmed.slice(lastColon + 1);
      if (/^\d+$/.test(portPart)) {
        attempts.add(`http://[${hostPart}]:${portPart}`);
        attempts.add(`ws://[${hostPart}]:${portPart}`);
      }
    }
  }

  for (const candidate of attempts) {
    try {
      return new URL(candidate);
    } catch {
      continue;
    }
  }

  return undefined;
}

function parseHostPort(value: string): { host?: string; port?: string } {
  const trimmed = value.trim();
  if (!trimmed) return {};
  if (/^[a-zA-Z][a-zA-Z0-9+.-]*:/.test(trimmed)) {
    return {};
  }

  if (trimmed.startsWith("[") && trimmed.includes("]")) {
    const end = trimmed.indexOf("]");
    const host = trimmed.slice(1, end);
    const rest = trimmed.slice(end + 1);
    const port = rest.startsWith(":") ? rest.slice(1) : undefined;
    return { host, port };
  }

  const colonCount = (trimmed.match(/:/g) ?? []).length;
  if (colonCount === 1) {
    const [host, port] = trimmed.split(":");
    if (host) {
      return { host, port: port?.trim() ? port : undefined };
    }
  }

  return { host: trimmed };
}

function wrapIpv6Host(host: string): string {
  if (host.startsWith("[") && host.endsWith("]")) {
    return host;
  }
  return host.includes(":") ? `[${host}]` : host;
}

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
  const sanitizedHost = bootstrap?.host?.trim();
  const parsedHost = sanitizedHost ? tryParseUrl(sanitizedHost) : undefined;
  const manualHost = sanitizedHost
    ? parseHostPort(sanitizedHost)
    : { host: undefined, port: undefined };

  const locationProtocol = normaliseProtocol(protocol) ?? "ws";
  const hostProtocol = normaliseProtocol(parsedHost?.protocol);
  const configuredProtocol = normaliseProtocol(bootstrap?.protocol);
  const finalProtocol = configuredProtocol ?? hostProtocol ?? locationProtocol;

  const locationPort = typeof port === "string" ? port.trim() : "";
  type PortSource = "bootstrap" | "host" | "location" | "default";
  let finalPortSource: PortSource = "default";
  let finalPort: string | undefined;

  const portCandidates: Array<{ value?: string; source: PortSource }> = [
    { value: bootstrap?.port, source: "bootstrap" },
    { value: parsedHost?.port, source: "host" },
    { value: manualHost.port, source: "host" },
    { value: locationPort, source: "location" },
  ];

  for (const candidate of portCandidates) {
    const trimmed = candidate.value?.trim();
    if (trimmed) {
      finalPort = trimmed;
      finalPortSource = candidate.source;
      break;
    }
  }

  if (
    !finalPort ||
    (finalPortSource === "location" && DEV_SERVER_PORTS.has(finalPort))
  ) {
    finalPort = "8088";
    finalPortSource = "default";
  }

  const hostCandidate = parsedHost?.hostname ?? manualHost.host ?? sanitizedHost ?? hostname;
  const trimmedHost = hostCandidate?.trim() || hostname;
  const safeHost = wrapIpv6Host(trimmedHost);
  const cockpitUrl = new URL(`${finalProtocol}://${safeHost}`);
  if (finalPort) {
    cockpitUrl.port = finalPort;
  }
  cockpitUrl.pathname = "/ws";
  return cockpitUrl.href;
}

export const __test__ = {
  defaultCockpitUrl,
  readBootstrappedConfig,
};
