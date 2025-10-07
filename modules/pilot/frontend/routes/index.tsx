import { define } from "../utils.ts";

interface ModuleLink {
  slug: string;
  name: string;
  description: string;
}

const modules: ModuleLink[] = [
  {
    slug: "pilot",
    name: "Pilot Module",
    description:
      "Status dashboard for the cockpit backend and websocket bridge.",
  },
  {
    slug: "imu",
    name: "IMU Module",
    description:
      "Read-only telemetry stream with orientation and velocity vectors.",
  },
  {
    slug: "foot",
    name: "Foot Module",
    description:
      "Send manual commands and monitor Create drive base telemetry and faults.",
  },
  {
    slug: "eye",
    name: "Eye Module",
    description:
      "Stream Kinect RGB-D telemetry for situational awareness.",
  },
];

export default define.route(() => {
  return (
    <section class="content">
      <h1>Psyched Pilot</h1>
      <p>Operator console for module diagnostics.</p>
      <ul class="module-list">
        {modules.map((module) => (
          <li class="module-list__item" key={module.slug}>
            <h2>{module.name}</h2>
            <p>{module.description}</p>
            <a href={`/modules/${module.slug}`}>Open module</a>
          </li>
        ))}
      </ul>
    </section>
  );
});
