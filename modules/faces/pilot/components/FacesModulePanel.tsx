import ModuleOverview from "../../../pilot/pilot/components/ModuleOverview.tsx";

const DESCRIPTION = "Face detection pipeline for presence and identity cues.";

/** Surface lifecycle controls for the Faces perception module. */
export default function FacesModulePanel() {
  return (
    <ModuleOverview
      moduleName="faces"
      title="Faces module"
      description={DESCRIPTION}
      accent="magenta"
    />
  );
}
