import ModuleOverview from "../../../pilot/pilot/components/ModuleOverview.tsx";

const DESCRIPTION =
  "Navigation stack coordinating localization and path planning.";

/** Lifecycle controls for the Nav autonomy module. */
export default function NavModulePanel() {
  return (
    <ModuleOverview
      moduleName="nav"
      title="Nav module"
      description={DESCRIPTION}
      accent="violet"
    />
  );
}
