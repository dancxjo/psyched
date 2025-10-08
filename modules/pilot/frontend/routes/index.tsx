import { Panel, Card, type Accent } from "@pilot/components/dashboard.tsx";
import { define } from "../utils.ts";

interface ModuleLink {
  slug: string;
  name: string;
  description: string;
  tone: Accent;
}

const modules: ModuleLink[] = [
  {
    slug: "pilot",
    name: "Pilot Module",
    description:
      "Status dashboard for the cockpit backend and websocket bridge.",
    tone: "amber",
  },
  {
    slug: "imu",
    name: "IMU Module",
    description:
      "Read-only telemetry stream with orientation and velocity vectors.",
    tone: "cyan",
  },
  {
    slug: "foot",
    name: "Foot Module",
    description:
      "Send manual commands and monitor Create drive base telemetry and faults.",
    tone: "teal",
  },
  {
    slug: "eye",
    name: "Eye Module",
    description: "Stream Kinect RGB-D telemetry for situational awareness.",
    tone: "violet",
  },
];

export default define.page(() => {
  return (
    <section class="content">
      <Panel
        title="Psyched Pilot"
        subtitle="Operator console for module diagnostics and orchestration"
        accent="violet"
      >
        <div class="panel-grid panel-grid--stretch">
          {modules.map((module) => (
            <Card
              key={module.slug}
              title={module.name}
              subtitle={module.description}
              tone={module.tone}
              footer={(
                <a
                  class="button button--primary"
                  href={`/modules/${module.slug}`}
                >
                  Open module
                </a>
              )}
            />
          ))}
        </div>
      </Panel>
    </section>
  );
});
