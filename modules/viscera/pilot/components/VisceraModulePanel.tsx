import ModuleOverview from "../../../pilot/pilot/components/ModuleOverview.tsx";

const DESCRIPTION =
  "System health monitors and feelers for onboard diagnostics.";

/** Provide lifecycle controls for the Viscera health monitoring module. */
export default function VisceraModulePanel() {
  return (
    <ModuleOverview
      moduleName="viscera"
      title="Viscera module"
      description={DESCRIPTION}
      accent="amber"
    />
  );
}
