import { assertEquals } from "$std/assert/mod.ts";
import { dirname, join } from "$std/path/mod.ts";

import { __test__, primaryNavigationLinks } from "./navigation.ts";

function withTempHostsDir(
  files: Record<string, string>,
  run: (dir: string) => void,
): void {
  const tempDir = Deno.makeTempDirSync();
  try {
    for (const [name, contents] of Object.entries(files)) {
      const path = join(tempDir, name);
      const parent = dirname(path);
      if (parent && parent !== tempDir) {
        Deno.mkdirSync(parent, { recursive: true });
      }
      Deno.writeTextFileSync(path, contents);
    }
    run(tempDir);
  } finally {
    Deno.removeSync(tempDir, { recursive: true });
  }
}

Deno.test("builds navigation from launched modules in the host manifest", () => {
  withTempHostsDir({
    "testhost.json": JSON.stringify(
      {
        host: {
          name: "testhost",
          modules: ["pilot"],
        },
        modules: {
          pilot: {},
          ear: { launch: true },
          imu: { launch: "yes" },
          foot: { launch: false },
          eye: { launch: { arguments: { camera_topic: "/camera" } } },
          nav: { launch: { enabled: true } },
        },
      },
      null,
      2,
    ),
  }, (dir) => {
    __test__.resetCache();
    const links = primaryNavigationLinks({
      hostname: "testhost",
      hostsDir: dir,
    });
    assertEquals(links, [
      { href: "/", label: "Home" },
      { href: "/modules/pilot", label: "Pilot" },
      { href: "/modules/ear", label: "Ear" },
      { href: "/modules/imu", label: "IMU" },
      { href: "/modules/eye", label: "Eye" },
      { href: "/modules/nav", label: "Nav" },
      { href: "/psh/host", label: "Host" },
      { href: "/psh/mod", label: "Modules" },
      { href: "/psh/srv", label: "Services" },
      { href: "/psh/sys", label: "Systemd" },
    ]);
  });
});

Deno.test("falls back to default navigation when host manifest is missing", () => {
  withTempHostsDir({}, (dir) => {
    __test__.resetCache();
    const links = primaryNavigationLinks({ hostname: "ghost", hostsDir: dir });
    assertEquals(links, [
      { href: "/", label: "Home" },
      { href: "/modules/pilot", label: "Pilot" },
      { href: "/psh/host", label: "Host" },
      { href: "/psh/mod", label: "Modules" },
      { href: "/psh/srv", label: "Services" },
      { href: "/psh/sys", label: "Systemd" },
    ]);
  });
});
