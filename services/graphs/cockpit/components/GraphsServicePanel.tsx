import { Card } from "@cockpit/components/dashboard.tsx";

import ServiceOverview from "../../../../modules/cockpit/cockpit/components/ServiceOverview.tsx";

export default function GraphsServicePanel() {
  return (
    <ServiceOverview
      service="graphs"
      title="Graphs service"
      subtitle="Knowledge graph backends"
      accent="magenta"
      description="Runs Neo4j and Memgraph for situational awareness graphs."
    >
      <Card title="Usage" tone="magenta">
        <p class="note">
          The graphs stack exposes Bolt endpoints on the provisioned host.
          Connect tooling such as Bloom or Cypher shell once the service badge
          reports <strong>Running</strong>.
        </p>
      </Card>
    </ServiceOverview>
  );
}
