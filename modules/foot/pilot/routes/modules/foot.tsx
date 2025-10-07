// @ts-nocheck -- ROS telemetry surface still relies on dynamic message shapes.
import FootControlPanel from "../../islands/FootControlPanelIsland.tsx";

export default function FootModulePage() {
  return (
    <section class="content">
      <FootControlPanel />
    </section>
  );
}
