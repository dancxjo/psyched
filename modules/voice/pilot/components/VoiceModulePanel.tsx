import type { TargetedEvent } from "preact";
import { useCallback, useEffect, useMemo, useReducer, useState } from "preact/hooks";

import { useCockpitTopic } from "@pilot/lib/cockpit.ts";

import { Card, CONNECTION_STATUS_LABELS, toneFromConnection } from "../../../pilot/pilot/components/dashboard.tsx";
import ModuleOverview from "../../../pilot/pilot/components/ModuleOverview.tsx";
import { formatRelativeTime } from "../../../pilot/frontend/lib/format.ts";

const DESCRIPTION = "Speech synthesis bridge coordinating the TTS service.";
const CONVERSATION_TOPIC = "/conversation";
const CONVERSATION_LIMIT = 20;
const REFRESH_INTERVAL_MS = 1_000;

type ConversationMessage = { data?: string | null };
type ConversationSource = "local" | "remote";

interface ConversationEntry {
  text: string;
  source: ConversationSource;
  receivedAt: number;
}

interface ConversationLogOptions {
  limit?: number;
  now?: () => number;
}

interface AppendConversationArgs {
  text: string;
  source: ConversationSource;
}

/**
 * Appends a conversational message to the rolling history, skipping consecutive
 * duplicates so operator echoes do not pollute the log.
 *
 * @example
 * ```ts
 * appendConversationLog([], { text: "Hello", source: "local" }, { now: () => 0 });
 * //=> [{ text: "Hello", source: "local", receivedAt: 0 }]
 * ```
 */
export function appendConversationLog(
  log: ConversationEntry[],
  entry: AppendConversationArgs,
  options: ConversationLogOptions = {},
): ConversationEntry[] {
  const trimmed = entry.text.trim();
  if (!trimmed) return log;

  const limit = options.limit ?? CONVERSATION_LIMIT;
  const now = options.now ?? (() => Date.now());
  const newest = log[0];
  if (newest && newest.text === trimmed) {
    return log;
  }

  const nextEntry: ConversationEntry = {
    text: trimmed,
    source: entry.source,
    receivedAt: now(),
  };
  const next = [nextEntry, ...log];
  return next.length > limit ? next.slice(0, limit) : next;
}

const SOURCE_LABELS: Record<ConversationSource, string> = {
  local: "Operator",
  remote: "Robot",
};

/** Lifecycle controls and TTS conversation surface for the Voice output module. */
export default function VoiceModulePanel() {
  const conversation = useCockpitTopic<ConversationMessage>(CONVERSATION_TOPIC, {
    replay: true,
  });
  const [log, setLog] = useState<ConversationEntry[]>([]);
  const [draft, setDraft] = useState("");
  const [, forceRefresh] = useReducer((tick: number) => tick + 1, 0);

  const appendMessage = useCallback(
    (message: string, source: ConversationSource) => {
      setLog((previous) =>
        appendConversationLog(previous, { text: message, source }, {
          limit: CONVERSATION_LIMIT,
          now: () => Date.now(),
        })
      );
    },
    [],
  );

  useEffect(() => {
    const payload = conversation.data?.data;
    if (typeof payload !== "string") {
      return;
    }
    appendMessage(payload, "remote");
  }, [conversation.data?.data, appendMessage]);

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

  const handleDraftChange = useCallback(
    (event: TargetedEvent<HTMLTextAreaElement, Event>) => {
      setDraft(event.currentTarget.value);
    },
    [],
  );

  const handleSubmit = useCallback(
    (event: Event) => {
      event.preventDefault();
      const message = draft.trim();
      if (!message) {
        return;
      }
      appendMessage(message, "local");
      conversation.publish({ data: message });
      setDraft("");
    },
    [draft, conversation, appendMessage],
  );

  const connectionLabel = CONNECTION_STATUS_LABELS[conversation.status] ?? "Unknown";
  const connectionTone = toneFromConnection(conversation.status);
  const lastUpdated = useMemo(
    () => formatRelativeTime(log[0]?.receivedAt ?? null),
    [log],
  );
  const canSend = draft.trim().length > 0;

  return (
    <ModuleOverview
      moduleName="voice"
      title="Voice module"
      description={DESCRIPTION}
      accent="cyan"
    >
      <Card
        title="Conversation"
        subtitle="Cockpit bridge to the TTS node"
        tone="cyan"
        icon={<ConversationIcon />}
        footer={
          <p class="note">
            Messages are forwarded to the ROS <code>{CONVERSATION_TOPIC}</code> topic.
          </p>
        }
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
            <dt>Last message</dt>
            <dd>{lastUpdated}</dd>
          </div>
        </dl>

        {log.length === 0 ? (
          <p class="placeholder">No conversation messages yet.</p>
        ) : (
          <ol class="log-list" aria-live="polite">
            {log.map((entry, index) => {
              const iso = new Date(entry.receivedAt).toISOString();
              return (
                <li class="log-list__item" key={`${entry.receivedAt}-${index}`}>
                  <div class="log-list__header">
                    <span class="chip">{SOURCE_LABELS[entry.source]}</span>
                    <time class="log-list__meta" dateTime={iso}>
                      {formatRelativeTime(entry.receivedAt)}
                    </time>
                  </div>
                  <p class="log-list__text">{entry.text}</p>
                </li>
              );
            })}
          </ol>
        )}

        <form class="conversation-form" onSubmit={handleSubmit}>
          <label class="sr-only" htmlFor="voice-draft">Send message</label>
          <textarea
            id="voice-draft"
            class="conversation-form__input"
            rows={3}
            placeholder="Type a phrase for Pete to speak"
            value={draft}
            onInput={handleDraftChange}
          />
          <div class="button-group">
            <button
              class="button button--primary"
              type="submit"
              disabled={!canSend}
            >
              Send to robot
            </button>
          </div>
        </form>
        {conversation.error && <p class="note note--alert">{conversation.error}</p>}
      </Card>
    </ModuleOverview>
  );
}

function ConversationIcon() {
  return (
    <svg viewBox="0 0 24 24" class="icon" aria-hidden="true">
      <path
        d="M4 6h16v9H7l-3 3z"
        fill="none"
        stroke="currentColor"
        stroke-width="2"
        stroke-linejoin="round"
      />
      <path
        d="M9 10h6M9 13h4"
        stroke="currentColor"
        stroke-width="2"
        stroke-linecap="round"
      />
    </svg>
  );
}

export const __test__ = { appendConversationLog };
