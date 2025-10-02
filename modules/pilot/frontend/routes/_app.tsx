import { Head } from "$fresh/runtime.ts";
import type { RouteContext } from "$fresh/server.ts";
import { define } from "../utils.ts";

export default define.app((_req: Request, ctx: RouteContext) => {
  const Component = ctx.Component;

  return (
    <html lang="en">
      <Head>
        <meta charSet="utf-8" />
        <meta
          name="viewport"
          content="width=device-width, initial-scale=1"
        />
        <title>Psyched Pilot</title>
        <link rel="stylesheet" href="/styles.css" />
      </Head>
      <body>
        <header class="site-header">
          <nav>
            <a href="/">Home</a>
            <a href="/modules/imu">IMU</a>
          </nav>
        </header>
        <main class="site-main">
          <Component />
        </main>
      </body>
    </html>
  );
});
