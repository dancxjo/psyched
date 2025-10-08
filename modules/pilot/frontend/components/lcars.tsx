import type { ComponentChildren } from "preact";

import type { ConnectionStatus } from "../lib/cockpit.ts";

/** Human friendly labels for cockpit connection states. */
export const CONNECTION_STATUS_LABELS: Readonly<
  Record<ConnectionStatus, string>
> = {
  idle: "Idle",
  connecting: "Connecting",
  open: "Connected",
  closed: "Disconnected",
  error: "Error",
} as const;

/** Badge accent used throughout the LCARS themed dashboard. */
export type LcarsBadgeTone = "neutral" | "info" | "ok" | "warn" | "danger";

/** Accent palette for panels and cards. */
export type LcarsAccent = "amber" | "teal" | "magenta" | "cyan" | "violet";

/**
 * Maps a websocket status to the colour palette used by LCARS badges.
 *
 * @example
 * ```ts
 * const tone = toneFromConnection("open"); // "ok"
 * ```
 */
export function toneFromConnection(
  status: ConnectionStatus | undefined,
): LcarsBadgeTone {
  switch (status) {
    case "open":
      return "ok";
    case "connecting":
      return "info";
    case "idle":
      return "neutral";
    case "closed":
      return "warn";
    case "error":
      return "danger";
    default:
      return "neutral";
  }
}

export interface LcarsBadgeProps {
  label: string;
  tone?: LcarsBadgeTone;
  pulse?: boolean;
}

/**
 * Compact pill-style badge suited for status readouts.
 */
export function LcarsBadge({
  label,
  tone = "neutral",
  pulse = false,
}: LcarsBadgeProps) {
  return (
    <span
      class={`lcars-badge lcars-badge--${tone}${
        pulse ? " lcars-badge--pulse" : ""
      }`}
    >
      {label}
    </span>
  );
}

export interface LcarsPanelProps {
  title: string;
  subtitle?: string;
  accent?: LcarsAccent;
  badges?: Array<{ label: string; tone?: LcarsBadgeTone; pulse?: boolean }>;
  actions?: ComponentChildren;
  children?: ComponentChildren;
}

/**
 * Primary container component for LCARS-styled dashboards. It ensures the
 * header, accent bar, and badges are consistently laid out across modules.
 */
export function LcarsPanel({
  title,
  subtitle,
  accent = "amber",
  badges = [],
  actions,
  children,
}: LcarsPanelProps) {
  return (
    <article class="lcars-panel" data-tone={accent}>
      <header class="lcars-panel__header">
        <div class="lcars-panel__heading">
          <span class="lcars-panel__accent" aria-hidden="true" />
          <div>
            <h1 class="lcars-panel__title">{title}</h1>
            {subtitle && <p class="lcars-panel__subtitle">{subtitle}</p>}
          </div>
        </div>
        <div class="lcars-panel__header-meta">
          {badges.length > 0 && (
            <div class="lcars-panel__badges">
              {badges.map((badge) => (
                <LcarsBadge
                  key={`${badge.label}-${badge.tone ?? "neutral"}`}
                  label={badge.label}
                  tone={badge.tone}
                  pulse={badge.pulse}
                />
              ))}
            </div>
          )}
          {actions && <div class="lcars-panel__actions">{actions}</div>}
        </div>
      </header>
      {children && <div class="lcars-panel__body">{children}</div>}
    </article>
  );
}

export interface LcarsCardProps {
  title: string;
  subtitle?: string;
  tone?: LcarsAccent | "neutral";
  icon?: ComponentChildren;
  footer?: ComponentChildren;
  actions?: ComponentChildren;
  children?: ComponentChildren;
}

/**
 * Content card nested inside a LCARS panel. Cards automatically inherit the
 * accent colour while keeping a consistent padding and border radius.
 */
export function LcarsCard({
  title,
  subtitle,
  tone = "neutral",
  icon,
  footer,
  actions,
  children,
}: LcarsCardProps) {
  return (
    <section class="lcars-card" data-tone={tone}>
      <header class="lcars-card__header">
        {icon && <span class="lcars-card__icon" aria-hidden="true">{icon}
        </span>}
        <div>
          <h2 class="lcars-card__title">{title}</h2>
          {subtitle && <p class="lcars-card__subtitle">{subtitle}</p>}
        </div>
        {actions && <div class="lcars-card__actions">{actions}</div>}
      </header>
      {children && <div class="lcars-card__body">{children}</div>}
      {footer && <footer class="lcars-card__footer">{footer}</footer>}
    </section>
  );
}
