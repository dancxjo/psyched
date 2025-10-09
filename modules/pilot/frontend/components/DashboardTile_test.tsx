import { assertStringIncludes } from "$std/assert/assert_string_includes.ts";
import render from "preact-render-to-string";

import DashboardTile from "./DashboardTile.tsx";

function OverlayStub() {
  return <div data-testid="overlay">ready</div>;
}

Deno.test("dashboard tiles render their overlays open by default", () => {
  const markup = render(
    <DashboardTile
      name="example"
      title="Example"
      description="Demo tile"
      kind="module"
      accent="teal"
      href="/modules/example"
      overlay={OverlayStub}
    />,
  );

  assertStringIncludes(
    markup,
    "<details open class=\"dashboard-tile__details\"",
  );
});
