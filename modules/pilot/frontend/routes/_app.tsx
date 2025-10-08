import { define } from "../utils.ts";

export default define.page(function App({ Component, state }) {
  const { cockpit } = state;
  return (
    <html lang="en">
      <head>
        <meta charSet="utf-8" />
        <meta name="viewport" content="width=device-width, initial-scale=1" />
        <title>Psyched Pilot</title>
        <link rel="stylesheet" href="/styles.css" />
      </head>
      <body
        data-cockpit-host={cockpit?.host}
        data-cockpit-port={cockpit?.port}
        data-cockpit-protocol={cockpit?.protocol}
        data-cockpit-url={cockpit?.url}
      >
        <header class="site-header">
          <div class="site-header__inner">
            <a class="site-brand" href="/">Psyched Pilot</a>
            <nav class="site-nav" aria-label="Primary navigation">
              <ul class="site-nav__list">
                <li><a href="/">Home</a></li>
                <li><a href="/modules/pilot">Pilot</a></li>
                <li><a href="/modules/imu">IMU</a></li>
                <li><a href="/modules/foot">Foot</a></li>
                <li><a href="/modules/eye">Eye</a></li>
                <li><a href="/psh/host">Host</a></li>
                <li><a href="/psh/mod">Modules</a></li>
                <li><a href="/psh/srv">Services</a></li>
                <li><a href="/psh/sys">Systemd</a></li>
              </ul>
            </nav>
          </div>
        </header>
        <main class="site-main">
          <Component />
        </main>
      </body>
    </html>
  );
});
