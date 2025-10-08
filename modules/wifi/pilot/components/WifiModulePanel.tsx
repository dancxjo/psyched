import ModuleOverview from "../../../pilot/pilot/components/ModuleOverview.tsx";

const DESCRIPTION = "Wireless connectivity tooling and diagnostics for Pete.";

/** Lifecycle controls for the Wi-Fi connectivity module. */
export default function WifiModulePanel() {
  return (
    <ModuleOverview
      moduleName="wifi"
      title="Wi-Fi module"
      description={DESCRIPTION}
      accent="teal"
    />
  );
}
