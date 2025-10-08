import { useCallback, useEffect, useMemo, useState } from "preact/hooks";

import type { BadgeTone } from "@pilot/components/dashboard.tsx";

export type ModuleStatusRecord = {
  name: string;
  status: "running" | "stopped";
  pid?: number;
};

export type ServiceStatusRecord = {
  name: string;
  status: "running" | "stopped" | "error";
  description?: string;
};

type ResourceKind = "module" | "service";

type StatusRecord = ModuleStatusRecord | ServiceStatusRecord;

type StatusResponse<T> = {
  ok: boolean;
  statuses?: T[];
  error?: string;
};

interface StatusCache<T extends StatusRecord> {
  data: T[] | null;
  promise: Promise<T[]> | null;
  timestamp: number;
}

const DEFAULT_REFRESH_INTERVAL_MS = 15_000;

const moduleCache: StatusCache<ModuleStatusRecord> = {
  data: null,
  promise: null,
  timestamp: 0,
};

const serviceCache: StatusCache<ServiceStatusRecord> = {
  data: null,
  promise: null,
  timestamp: 0,
};

const MODULE_LABELS: Record<ModuleStatusRecord["status"], string> = {
  running: "Running",
  stopped: "Stopped",
};

const MODULE_TONES: Record<ModuleStatusRecord["status"], BadgeTone> = {
  running: "ok",
  stopped: "warn",
};

const SERVICE_LABELS: Record<ServiceStatusRecord["status"], string> = {
  running: "Running",
  stopped: "Stopped",
  error: "Error",
};

const SERVICE_TONES: Record<ServiceStatusRecord["status"], BadgeTone> = {
  running: "ok",
  stopped: "warn",
  error: "danger",
};

function cacheFor(kind: ResourceKind): StatusCache<StatusRecord> {
  return kind === "module"
    ? moduleCache as StatusCache<StatusRecord>
    : serviceCache as StatusCache<StatusRecord>;
}

function endpointFor(kind: ResourceKind): string {
  return kind === "module" ? "/api/psh/mod/list" : "/api/psh/srv/list";
}

async function fetchStatuses<T extends StatusRecord>(
  kind: ResourceKind,
  force = false,
): Promise<T[]> {
  const cache = cacheFor(kind) as StatusCache<T>;
  if (!force && cache.data) {
    return cache.data;
  }
  if (!force && cache.promise) {
    return cache.promise;
  }

  if (force) {
    cache.data = null;
  }

  const request = fetch(endpointFor(kind))
    .then(async (response): Promise<T[]> => {
      if (!response.ok) {
        throw new Error(`HTTP ${response.status}`);
      }
      const payload = await response.json() as StatusResponse<T>;
      if (!payload.ok || !payload.statuses) {
        throw new Error(payload.error ?? "Unknown status response");
      }
      cache.data = payload.statuses;
      cache.timestamp = Date.now();
      return payload.statuses;
    })
    .finally(() => {
      cache.promise = null;
    });

  cache.promise = request;
  return request;
}

interface UsePshStatusOptions {
  /** Milliseconds between automatic refreshes. Defaults to 15 seconds. */
  refreshIntervalMs?: number;
}

export interface UsePshStatusResult {
  status?: string;
  label: string;
  tone: BadgeTone;
  loading: boolean;
  description?: string;
  error?: string;
  lastUpdated?: number;
  refresh: () => void;
}

function mapModuleBadge(
  status: ModuleStatusRecord | undefined,
): Pick<UsePshStatusResult, "label" | "tone" | "description" | "status"> {
  if (!status) {
    return {
      status: undefined,
      label: "Not found",
      tone: "danger",
      description: "Module is not provisioned",
    };
  }

  return {
    status: status.status,
    label: MODULE_LABELS[status.status] ?? "Unknown",
    tone: MODULE_TONES[status.status] ?? "neutral",
    description: status.pid ? `PID ${status.pid}` : undefined,
  };
}

function mapServiceBadge(
  status: ServiceStatusRecord | undefined,
): Pick<UsePshStatusResult, "label" | "tone" | "description" | "status"> {
  if (!status) {
    return {
      status: undefined,
      label: "Not found",
      tone: "danger",
      description: "Service manifest missing",
    };
  }

  return {
    status: status.status,
    label: SERVICE_LABELS[status.status] ?? "Unknown",
    tone: SERVICE_TONES[status.status] ?? "neutral",
    description: status.description,
  };
}

/**
 * Reactively read lifecycle information for a module or service managed by
 * `psh`.
 *
 * The hook polls the appropriate `/api/psh/(mod|srv)/list` endpoint and caches
 * the response so concurrent dashboard widgets do not trigger duplicate
 * requests. Consumers receive the current status label, badge tone, and a
 * refresh callback they can surface in their UI.
 *
 * @example
 * ```tsx
 * const status = usePshStatus("service", "asr");
 * return <Badge label={status.label} tone={status.tone} />;
 * ```
 */
export function usePshStatus(
  kind: ResourceKind,
  name: string,
  options: UsePshStatusOptions = {},
): UsePshStatusResult {
  const [state, setState] = useState<UsePshStatusResult>(() => ({
    status: undefined,
    label: "Loading",
    tone: "info",
    loading: true,
    description: undefined,
    error: undefined,
    lastUpdated: undefined,
    refresh: () => undefined,
  }));

  const refreshInterval = options.refreshIntervalMs ??
    DEFAULT_REFRESH_INTERVAL_MS;

  const load = useCallback(async (force = false) => {
    setState((previous) => ({ ...previous, loading: true, error: undefined }));
    try {
      const statuses = await fetchStatuses<StatusRecord>(kind, force);
      if (kind === "module") {
        const match = (statuses as ModuleStatusRecord[])
          .find((entry) => entry.name === name);
        const mapped = mapModuleBadge(match);
        setState({
          ...mapped,
          loading: false,
          error: undefined,
          lastUpdated: Date.now(),
          refresh: () => {
            void load(true);
          },
        });
        return;
      }

      const match = (statuses as ServiceStatusRecord[])
        .find((entry) => entry.name === name);
      const mapped = mapServiceBadge(match);
      setState({
        ...mapped,
        loading: false,
        error: undefined,
        lastUpdated: Date.now(),
        refresh: () => {
          void load(true);
        },
      });
    } catch (error) {
      setState({
        status: undefined,
        label: "Unavailable",
        tone: "danger",
        loading: false,
        description: undefined,
        error: String(error),
        lastUpdated: Date.now(),
        refresh: () => {
          void load(true);
        },
      });
    }
  }, [kind, name]);

  useEffect(() => {
    let cancelled = false;

    const run = async () => {
      await load();
    };

    void run();

    let timer: number | undefined;
    if (Number.isFinite(refreshInterval) && refreshInterval > 0) {
      timer = window.setInterval(() => {
        if (!cancelled) {
          void load(true);
        }
      }, refreshInterval);
    }

    return () => {
      cancelled = true;
      if (timer !== undefined) {
        window.clearInterval(timer);
      }
    };
  }, [load, refreshInterval]);

  return useMemo<UsePshStatusResult>(() => ({
    ...state,
    refresh: () => {
      void load(true);
    },
  }), [load, state]);
}
