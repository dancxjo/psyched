import { Card } from "@pilot/components/dashboard.tsx";

import ServiceOverview from "../../../../modules/pilot/pilot/components/ServiceOverview.tsx";

export default function LlmServicePanel() {
  return (
    <ServiceOverview
      service="llm"
      title="LLM service"
      subtitle="Conversational model runtime"
      accent="violet"
      description="Hosts large language model runtimes for dialogue orchestration."
    >
      <Card title="Deployment" tone="violet">
        <p class="note">
          The runtime exposes an OpenAI-compatible API once the container is up.
          Point the voice or brain modules to the forwarded port to enable
          conversations.
        </p>
      </Card>
    </ServiceOverview>
  );
}
