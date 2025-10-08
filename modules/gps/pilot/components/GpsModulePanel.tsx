import ModuleOverview from "../../../pilot/pilot/components/ModuleOverview.tsx";

const DESCRIPTION = "GNSS receiver integration for Pete's localization stack.";

/** Lifecycle controls for the GPS hardware integration module. */
export default function GpsModulePanel() {
  return (
    <ModuleOverview
      moduleName="gps"
      title="GPS module"
      description={DESCRIPTION}
      accent="amber"
    />
  );
}
