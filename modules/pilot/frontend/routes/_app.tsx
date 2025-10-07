import { define } from "../utils.ts";

export default define.page(function App({ Component }) {
  return (
    <html lang="en">
      <head>
        <meta charSet="utf-8" />
        <meta name="viewport" content="width=device-width, initial-scale=1" />
        <title>Psyched Pilot</title>
        <link rel="stylesheet" href="/styles.css" />
      </head>
      <body>
        <header class="site-header">
          <nav>
            <a href="/">Home</a>
            <a href="/modules/pilot">Pilot</a>
            <a href="/modules/imu">IMU</a>
            <a href="/modules/foot">Foot</a>
            <a href="/modules/eye">Eye</a>
            <a href="/psh/host">Host</a>
            <a href="/psh/mod">Modules</a>
            <a href="/psh/srv">Services</a>
            <a href="/psh/sys">Systemd</a>
          </nav>
        </header>
        <main class="site-main">
          <Component />
        </main>
      </body>
    </html>
  );
});
