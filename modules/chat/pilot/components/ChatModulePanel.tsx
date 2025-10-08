import ModuleOverview from "../../../pilot/pilot/components/ModuleOverview.tsx";

const DESCRIPTION =
  "Conversational agent orchestrating Pete's dialogue skills.";

/**
 * Lifecycle controls for the Chat module, delegating to {@link ModuleOverview}.
 */
export default function ChatModulePanel() {
  return (
    <ModuleOverview
      moduleName="chat"
      title="Chat module"
      description={DESCRIPTION}
      accent="violet"
    />
  );
}
