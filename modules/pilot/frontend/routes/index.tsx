import { type Handlers, type PageProps } from "$fresh/server.ts";

interface ModuleLink {
  slug: string;
  name: string;
  description: string;
}

interface IndexData {
  modules: ModuleLink[];
}

export const handler: Handlers<IndexData> = {
  GET(_req, ctx) {
    return ctx.render({
      modules: [
        {
          slug: "imu",
          name: "IMU Module",
          description: "Read-only telemetry stream with orientation and velocity vectors.",
        },
      ],
    });
  },
};

export default function IndexPage(props: PageProps<IndexData>) {
  const { modules } = props.data;
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
}
