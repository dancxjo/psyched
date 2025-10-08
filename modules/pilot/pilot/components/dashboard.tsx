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

/** Badge accent used throughout the cockpit dashboard. */
export type BadgeTone = "neutral" | "info" | "ok" | "warn" | "danger";

/** Accent palette for panels and cards. */
export type Accent = "amber" | "teal" | "magenta" | "cyan" | "violet";

/**
 * Maps a websocket status to the colour palette used by cockpit badges.
 *
 * @example
 * ```ts
 * const tone = toneFromConnection("open"); // "ok"
 * ```
 */
export function toneFromConnection(
  status: ConnectionStatus | undefined,
): BadgeTone {
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

export interface BadgeProps {
  label: string;
  tone?: BadgeTone;
  pulse?: boolean;
}

/**
 * Compact pill-style badge suited for status readouts.
 */
export function Badge({
  label,
  tone = "neutral",
  pulse = false,
}: BadgeProps) {
  return (
    <span
      class={`status-badge status-badge--${tone}${
        pulse ? " status-badge--pulse" : ""
      }`}
    >
      {label}
    </span>
  );
}

export interface PanelProps {
  title: string;
  subtitle?: string;
  accent?: Accent;
  badges?: Array<{ label: string; tone?: BadgeTone; pulse?: boolean }>;
  actions?: ComponentChildren;
  children?: ComponentChildren;
}

/**
 * Primary container component for cockpit-styled dashboards. It ensures the
 * header, accent bar, and badges are consistently laid out across modules.
 */
export function Panel({
  title,
  subtitle,
  accent = "amber",
  badges = [],
  actions,
  children,
}: PanelProps) {
  return (
    <article class="dashboard-panel" data-tone={accent}>
      <header class="dashboard-panel__header">
        <div class="dashboard-panel__heading">
          <span class="dashboard-panel__accent" aria-hidden="true" />
          <div>
            <h1 class="dashboard-panel__title">{title}</h1>
            {subtitle && <p class="dashboard-panel__subtitle">{subtitle}</p>}
          </div>
        </div>
        <div class="dashboard-panel__meta">
          {badges.length > 0 && (
            <div class="dashboard-panel__badges">
              {badges.map((badge) => (
                <Badge
                  key={`${badge.label}-${badge.tone ?? "neutral"}`}
                  label={badge.label}
                  tone={badge.tone}
                  pulse={badge.pulse}
                />
              ))}
            </div>
          )}
          {actions && <div class="dashboard-panel__actions">{actions}</div>}
        </div>
      </header>
      {children && <div class="dashboard-panel__body">{children}</div>}
    </article>
  );
}

export interface CardProps {
  title: string;
  subtitle?: string;
  tone?: Accent | "neutral";
  icon?: ComponentChildren;
  footer?: ComponentChildren;
  actions?: ComponentChildren;
  children?: ComponentChildren;
}

/**
 * Content card nested inside a cockpit panel. Cards automatically inherit the
 * accent colour while keeping a consistent padding and border radius.
 */
export function Card({
  title,
  subtitle,
  tone = "neutral",
  icon,
  footer,
  actions,
  children,
}: CardProps) {
  return (
    <section class="card" data-tone={tone}>
      <header class="card__header">
        {icon && <span class="card__icon" aria-hidden="true">{icon}</span>}
        <div>
          <h2 class="card__title">{title}</h2>
          {subtitle && <p class="card__subtitle">{subtitle}</p>}
        </div>
        {actions && <div class="card__actions">{actions}</div>}
      </header>
      {children && <div class="card__body">{children}</div>}
      {footer && <footer class="card__footer">{footer}</footer>}
    </section>
  );
}
