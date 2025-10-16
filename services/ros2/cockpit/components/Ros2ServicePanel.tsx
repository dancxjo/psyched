import { Card } from "@cockpit/components/dashboard.tsx";

import ServiceOverview from "../../../../modules/cockpit/cockpit/components/ServiceOverview.tsx";

export default function Ros2ServicePanel() {
  return (
    <ServiceOverview
      service="ros2"
      title="ROS 2 bridge"
      subtitle="Containerised workspace utilities"
      accent="amber"
      description="Bootstraps auxiliary ROS 2 tooling and remote shells."
    >
      <Card title="Notes" tone="amber">
        <p class="note">
          Use <code>psh srv up ros2</code>{" "}
          before building modules that depend on Dockerised tooling such as{" "}
          <code>rosdep</code> or auxiliary bridge nodes.
        </p>
      </Card>
    </ServiceOverview>
  );
}
