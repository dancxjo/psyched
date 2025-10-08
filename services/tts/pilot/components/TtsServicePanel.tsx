import { Card } from "@pilot/components/dashboard.tsx";

import ServiceOverview from "../../../../modules/pilot/pilot/components/ServiceOverview.tsx";

export default function TtsServicePanel() {
  return (
    <ServiceOverview
      service="tts"
      title="TTS service"
      subtitle="Voice synthesis backend"
      accent="cyan"
      description="Provides text-to-speech voices for the voice module."
    >
      <Card title="Voices" tone="cyan">
        <p class="note">
          Load custom voices by mounting additional models into the compose
          stack. Refresh after syncing assets to see status updates.
        </p>
      </Card>
    </ServiceOverview>
  );
}
