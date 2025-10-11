import { useCallback, useEffect, useMemo, useReducer, useState } from "preact/hooks";

import { useCockpitTopic } from "@pilot/lib/cockpit.ts";

import { Card, CONNECTION_STATUS_LABELS, toneFromConnection } from "../../../pilot/pilot/components/dashboard.tsx";
import ModuleOverview from "../../../pilot/pilot/components/ModuleOverview.tsx";
import { formatRelativeTime } from "../../../pilot/frontend/lib/format.ts";

const DESCRIPTION = "Microphone ingestion and speech recognition interface.";
const TRANSCRIPT_TOPIC = "/audio/transcript/final";
const TRANSCRIPT_LIMIT = 12;
const REFRESH_INTERVAL_MS = 1_000;

type TranscriptMessage = { data?: string | null };

interface TranscriptEntry {
  text: string;
  receivedAt: number;
}

interface TranscriptLogOptions {
  limit?: number;
  now?: () => number;
}

/**
 * Adds a transcript line to the rolling log while preventing consecutive
 * duplicates and keeping the newest entry first.
 *
 * @example
 * ```ts
 * appendTranscriptLog([], "Hello", { now: () => 0 });
 * //=> [{ text: "Hello", receivedAt: 0 }]
 * ```
 */
export function appendTranscriptLog(
  log: TranscriptEntry[],
  text: string,
  options: TranscriptLogOptions = {},
): TranscriptEntry[] {
  const trimmed = text.trim();
  if (!trimmed) return log;

  const limit = options.limit ?? TRANSCRIPT_LIMIT;
  const now = options.now ?? (() => Date.now());
  const newest = log[0];
  if (newest && newest.text === trimmed) {
    return log;
  }

  const entry: TranscriptEntry = {
    text: trimmed,
    receivedAt: now(),
  };
  const next = [entry, ...log];
  return next.length > limit ? next.slice(0, limit) : next;
}

/** Present lifecycle controls for the Ear audio capture module. */
export default function EarModulePanel() {
  const transcript = useCockpitTopic<TranscriptMessage>(TRANSCRIPT_TOPIC, {
    replay: true,
  });
  const [log, setLog] = useState<TranscriptEntry[]>([]);
  const [, forceRefresh] = useReducer((tick: number) => tick + 1, 0);

  const addTranscript = useCallback((text: string) => {
    setLog((previous) =>
      appendTranscriptLog(previous, text, {
        now: () => Date.now(),
        limit: TRANSCRIPT_LIMIT,
      })
    );
  }, []);

  useEffect(() => {
    const message = transcript.data?.data;
    if (typeof message !== "string") {
      return;
    }
    addTranscript(message);
  }, [transcript.data?.data, addTranscript]);

  useEffect(() => {
    if (
      typeof globalThis.setInterval !== "function" ||
      typeof globalThis.clearInterval !== "function"
    ) {
      return;
    }
    const timer = globalThis.setInterval(() => {
      forceRefresh();
    }, REFRESH_INTERVAL_MS);
    return () => globalThis.clearInterval(timer);
  }, [forceRefresh]);

  const connectionLabel = CONNECTION_STATUS_LABELS[transcript.status] ?? "Unknown";
  const connectionTone = toneFromConnection(transcript.status);
  const lastUpdated = useMemo(
    () => formatRelativeTime(log[0]?.receivedAt ?? null),
    [log]
  );

  return (
    <ModuleOverview
      moduleName="ear"
      title="Ear module"
      description={DESCRIPTION}
      accent="cyan"
    >
      <Card
        title="Transcripts"
        subtitle="Final recogniser output"
        tone="cyan"
        icon={<TranscriptIcon />}
      >
        <dl class="stat-list">
          <div class="stat-list__item">
            <dt>Status</dt>
            <dd>
              <span class={`status-badge status-badge--${connectionTone}`}>
                {connectionLabel}
              </span>
            </dd>
          </div>
          <div class="stat-list__item">
            <dt>Last update</dt>
            <dd>{lastUpdated}</dd>
          </div>
        </dl>
        {log.length === 0 ? (
          <p class="placeholder">No transcripts received yet.</p>
        ) : (
          <ol class="log-list" aria-live="polite">
            {log.map((entry, index) => {
              const iso = new Date(entry.receivedAt).toISOString();
              return (
                <li class="log-list__item" key={`${entry.receivedAt}-${index}`}>
                  <p class="log-list__text">{entry.text}</p>
                  <time class="log-list__meta" dateTime={iso}>
                    {formatRelativeTime(entry.receivedAt)}
                  </time>
                </li>
              );
            })}
          </ol>
        )}
        {transcript.error && <p class="note note--alert">{transcript.error}</p>}
      </Card>
    </ModuleOverview>
  );
}

function TranscriptIcon() {
  return (
    <svg viewBox="0 0 24 24" class="icon" aria-hidden="true">
      <path
        d="M5 4h10l4 4v12H5z"
        fill="none"
        stroke="currentColor"
        stroke-width="2"
        stroke-linejoin="round"
      />
      <path
        d="M9 12h6M9 16h3"
        stroke="currentColor"
        stroke-width="2"
        stroke-linecap="round"
      />
    </svg>
  );
}

export const __test__ = { appendTranscriptLog };
