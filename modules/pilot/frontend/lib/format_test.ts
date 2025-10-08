import { describe, it } from "$std/testing/bdd.ts";
import { assertEquals } from "$std/testing/asserts.ts";

import {
  formatBytes,
  formatNullableNumber,
  formatRelativeTime,
} from "./format.ts";

describe("formatNullableNumber", () => {
  it("returns an em dash when the input is nullish", () => {
    assertEquals(formatNullableNumber(undefined), "—");
    assertEquals(formatNullableNumber(null), "—");
    assertEquals(formatNullableNumber(NaN), "—");
  });

  it("trims the value to the requested precision", () => {
    assertEquals(formatNullableNumber(1.2345), "1.23");
    assertEquals(formatNullableNumber(1.2349, { fractionDigits: 3 }), "1.235");
  });
});

describe("formatRelativeTime", () => {
  const now = Date.UTC(2024, 0, 1, 0, 0, 0);

  it("returns an em dash for missing timestamps", () => {
    assertEquals(formatRelativeTime(undefined, { now }), "—");
    assertEquals(formatRelativeTime(null, { now }), "—");
  });

  it("caps very recent timestamps to <1s", () => {
    assertEquals(formatRelativeTime(now - 100, { now }), "<1s");
  });

  it("reports seconds, minutes, hours, and days", () => {
    assertEquals(formatRelativeTime(now - 12_000, { now }), "12s");
    assertEquals(formatRelativeTime(now - 3 * 60 * 1000, { now }), "3m");
    assertEquals(formatRelativeTime(now - 2 * 60 * 60 * 1000, { now }), "2h");
    assertEquals(
      formatRelativeTime(now - 3 * 24 * 60 * 60 * 1000, { now }),
      "3d",
    );
  });
});

describe("formatBytes", () => {
  it("returns an em dash when the payload is missing", () => {
    assertEquals(formatBytes(undefined), "—");
    assertEquals(formatBytes(null as unknown as number), "—");
    assertEquals(formatBytes(NaN), "—");
  });

  it("formats values with IEC units", () => {
    assertEquals(formatBytes(512), "512 B");
    assertEquals(formatBytes(1_500), "1.5 KiB");
    assertEquals(formatBytes(2_620_000), "2.5 MiB");
  });
});
