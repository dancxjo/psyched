import ModuleOverview from "../../../pilot/pilot/components/ModuleOverview.tsx";

const DESCRIPTION = "Microphone ingestion and speech recognition interface.";

/** Present lifecycle controls for the Ear audio capture module. */
export default function EarModulePanel() {
  return (
    <ModuleOverview
      moduleName="ear"
      title="Ear module"
      description={DESCRIPTION}
      accent="cyan"
    />
  );
}
