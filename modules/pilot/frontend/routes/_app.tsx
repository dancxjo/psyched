import { CockpitProvider } from "@pilot/lib/cockpit.ts";
import { define } from "../utils.ts";

export default define.page(function App({ Component, state }) {
  const { cockpit, navigation } = state;
  const cockpitUrl = cockpit?.url?.trim();
  const navItems = navigation && navigation.length > 0 ? navigation : [
    { href: "/", label: "Home" },
    { href: "/modules/pilot", label: "Pilot" },
    { href: "/psh/host", label: "Host" },
    { href: "/psh/mod", label: "Modules" },
    { href: "/psh/srv", label: "Services" },
    { href: "/psh/sys", label: "Systemd" },
  ];
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
        <CockpitProvider
          options={cockpitUrl ? { url: cockpitUrl } : undefined}
        >
          <header class="site-header">
            <div class="site-header__inner">
              <a class="site-brand" href="/">Psyched Pilot</a>
              <nav class="site-nav" aria-label="Primary navigation">
                <ul class="site-nav__list">
                  {navItems.map((link) => (
                    <li key={link.href}>
                      <a href={link.href}>{link.label}</a>
                    </li>
                  ))}
                </ul>
              </nav>
            </div>
          </header>
          <main class="site-main">
            <Component />
          </main>
        </CockpitProvider>
      </body>
    </html>
  );
});
