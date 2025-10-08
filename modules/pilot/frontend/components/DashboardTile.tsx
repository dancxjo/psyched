import type { ComponentChildren, ComponentType } from "preact";

import { type Accent, Card } from "@pilot/components/dashboard.tsx";

import DashboardStatusBadge from "../islands/DashboardStatusBadge.tsx";

export interface DashboardTileProps {
  name: string;
  title: string;
  description: string;
  kind: "module" | "service";
  accent: Accent;
  href: string;
  overlay: ComponentType<unknown>;
  overlayProps?: Record<string, unknown>;
  ctaLabel?: string;
  helper?: ComponentChildren;
}

/**
 * Compact card that pairs lifecycle telemetry with a collapsible overlay.
 */
export default function DashboardTile({
  name,
  title,
  description,
  kind,
  accent,
  href,
  overlay: Overlay,
  overlayProps = {},
  ctaLabel,
  helper,
}: DashboardTileProps) {
  const label = ctaLabel ??
    (kind === "service" ? "Manage service" : "Open module");

  return (
    <Card
      title={title}
      subtitle={description}
      tone={accent}
      actions={<DashboardStatusBadge kind={kind} name={name} />}
    >
      {helper && <div class="dashboard-tile__helper">{helper}</div>}
      <details class="dashboard-tile__details">
        <summary class="dashboard-tile__summary">Show live overlay</summary>
        <div class="dashboard-tile__overlay">
          <Overlay {...overlayProps} />
        </div>
      </details>
      <footer class="dashboard-tile__footer">
        <a class="button button--small button--primary" href={href}>
          {label}
        </a>
      </footer>
    </Card>
  );
}
