import { Card } from "@pilot/components/dashboard.tsx";

import ServiceOverview from "../../../../modules/pilot/pilot/components/ServiceOverview.tsx";

export default function VectorsServicePanel() {
  return (
    <ServiceOverview
      service="vectors"
      title="Vector store"
      subtitle="Embedding memory backend"
      accent="teal"
      description="Maintains embeddings for retrieval augmented behaviours."
    >
      <Card title="Health" tone="teal">
        <p class="note">
          Monitor disk usage and ingestion health from this overlay. Use
          <code>psh srv up vectors</code>{" "}
          before triggering knowledge base synchronisation jobs.
        </p>
      </Card>
    </ServiceOverview>
  );
}
