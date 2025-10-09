import { Card, Panel } from "@pilot/components/dashboard.tsx";

import DashboardTile from "../components/DashboardTile.tsx";
import {
  moduleTilesForHost,
  serviceTilesForHost,
} from "../lib/dashboard/tiles.ts";
import { define } from "../utils.ts";

export default define.page(() => {
  const moduleTiles = moduleTilesForHost();
  const moduleCount = moduleTiles.length;
  const serviceTiles = serviceTilesForHost();
  const serviceCount = serviceTiles.length;

  return (
    <section class="content">
      <Panel
        title="Psyched cockpit"
        subtitle="Live system overview for Pete's modules and services"
        accent="violet"
      >
        <div class="panel-grid panel-grid--stretch">
          <Card title="Modules" subtitle="Managed by psh mod" tone="violet">
            <p class="note">
              {moduleCount} module{moduleCount === 1 ? "" : "s"}{" "}
              exported a dashboard overlay. Expand a tile below to monitor or
              control them without leaving the homepage.
            </p>
          </Card>
          <Card title="Services" subtitle="Managed by psh srv" tone="cyan">
            <p class="note">
              {serviceCount} service{serviceCount === 1 ? "" : "s"}{" "}
              expose live status from their docker compose stacks. Use the
              cockpit controls to refresh or jump straight into lifecycle
              tooling.
            </p>
          </Card>
        </div>
      </Panel>

      <Panel
        title="Module dashboards"
        subtitle="Compact overlays sourced from each module's pilot bundle"
        accent="teal"
      >
        <div class="dashboard-grid">
          {moduleTiles.map((tile) => (
            <DashboardTile key={tile.name} {...tile} />
          ))}
        </div>
      </Panel>

      <Panel
        title="Service dashboards"
        subtitle="Lifecycle telemetry for dockerised infrastructure"
        accent="magenta"
      >
        <div class="dashboard-grid">
          {serviceTiles.map((tile) => (
            <DashboardTile key={tile.name} {...tile} />
          ))}
        </div>
      </Panel>
    </section>
  );
});
