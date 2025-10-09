import { assertEquals } from "$std/assert/assert_equals.ts";
import render from "preact-render-to-string";
import { stub } from "$std/testing/mock.ts";

import * as statusModule from "@pilot/lib/psh_status.ts";
import DashboardStatusBadge, {
  DASHBOARD_STATUS_REFRESH_INTERVAL_MS,
} from "./DashboardStatusBadge.tsx";

Deno.test("dashboard status badge subscribes with live refresh interval", () => {
  const fakeStatus = {
    status: "running",
    label: "Running",
    tone: "ok" as const,
    loading: false,
    refresh: () => {},
  };

  let receivedInterval: number | undefined;
  const statusStub = stub(
    statusModule,
    "usePshStatus",
    (_kind, _name, options) => {
      receivedInterval = options?.refreshIntervalMs;
      return fakeStatus;
    },
  );

  try {
    render(<DashboardStatusBadge kind="module" name="pilot" />);
  } finally {
    statusStub.restore();
  }

  assertEquals(
    receivedInterval,
    DASHBOARD_STATUS_REFRESH_INTERVAL_MS,
  );
});
