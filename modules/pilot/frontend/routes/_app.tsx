import { Head } from "$fresh/runtime.ts";
import type { AppProps } from "$fresh/server.ts";

export default function App({ Component }: AppProps) {
  return (
    <>
      <Head>
        <meta charSet="utf-8" />
        <meta name="viewport" content="width=device-width, initial-scale=1" />
        <title>Psyched Pilot</title>
        <link rel="stylesheet" href="/styles.css" />
      </Head>
      <header class="site-header">
        <nav>
          <a href="/">Home</a>
          <a href="/modules/imu">IMU</a>
        </nav>
      </header>
      <main class="site-main">
        <Component />
      </main>
    </>
  );
}
