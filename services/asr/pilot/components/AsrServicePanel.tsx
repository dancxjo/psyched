import { Card } from "@pilot/components/dashboard.tsx";

import ServiceOverview from "../../../../modules/pilot/pilot/components/ServiceOverview.tsx";

export default function AsrServicePanel() {
  return (
    <ServiceOverview
      service="asr"
      title="ASR service"
      subtitle="Streaming Whisper transcription"
      accent="teal"
      description="Transforms operator audio into text events for the ear module."
    >
      <Card title="Tips" tone="teal">
        <p class="note">
          Ensure the ASR models are downloaded before startup. Provisioning via
          <code>psh srv setup asr</code> fetches the latest checkpoints.
        </p>
      </Card>
    </ServiceOverview>
  );
}
