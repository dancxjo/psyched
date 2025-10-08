import ModuleOverview from "../../../pilot/pilot/components/ModuleOverview.tsx";

const DESCRIPTION =
  "High-level intent planner orchestrating Pete's behaviours.";

/** Lifecycle controls for the Will behaviour coordination module. */
export default function WillModulePanel() {
  return (
    <ModuleOverview
      moduleName="will"
      title="Will module"
      description={DESCRIPTION}
      accent="magenta"
    />
  );
}
