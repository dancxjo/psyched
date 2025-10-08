import ModuleOverview from "../../../pilot/pilot/components/ModuleOverview.tsx";

const DESCRIPTION =
  "Semantic memory pipelines bridging vectors and graph stores.";

/** Show lifecycle controls for the Memory knowledge store module. */
export default function MemoryModulePanel() {
  return (
    <ModuleOverview
      moduleName="memory"
      title="Memory module"
      description={DESCRIPTION}
      accent="teal"
    />
  );
}
