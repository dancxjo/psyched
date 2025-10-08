import ModuleOverview from "../../../pilot/pilot/components/ModuleOverview.tsx";

const DESCRIPTION = "Speech synthesis bridge coordinating the TTS service.";

/** Lifecycle controls for the Voice output module. */
export default function VoiceModulePanel() {
  return (
    <ModuleOverview
      moduleName="voice"
      title="Voice module"
      description={DESCRIPTION}
      accent="cyan"
    />
  );
}
