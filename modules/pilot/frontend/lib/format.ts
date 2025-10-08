/**
 * Utility helpers for formatting telemetry values before rendering them in the
 * pilot UI. The helpers intentionally trade completeness for predictable
 * output that is easy to scan on a dashboard.
 */

export interface FormatNumberOptions {
  /** Number of fractional digits to display. Defaults to two. */
  fractionDigits?: number;
  /** Value to fall back to when the payload is nullish or NaN. */
  fallback?: string;
}

export interface FormatTemperaturePairOptions {
  /** Number of fractional digits to display for the Celsius value. */
  celsiusDigits?: number;
  /** Number of fractional digits to display for the Fahrenheit value. */
  fahrenheitDigits?: number;
  /** Placeholder used when the payload is nullish or not a finite number. */
  fallback?: string;
}

const DEFAULT_NUMBER_OPTIONS: Required<
  Pick<FormatNumberOptions, "fractionDigits" | "fallback">
> = {
  fractionDigits: 2,
  fallback: "—",
};

/**
 * Formats a numeric value while guarding against `null`, `undefined`, and
 * `NaN`. The output intentionally mirrors the cockpit dashboard style by
 * returning an em dash when the data is unavailable.
 *
 * @example
 * ```ts
 * formatNullableNumber(3.14159, { fractionDigits: 1 }); // "3.1"
 * formatNullableNumber(undefined); // "—"
 * ```
 */
export function formatNullableNumber(
  value: number | null | undefined,
  options: FormatNumberOptions = {},
): string {
  const { fractionDigits, fallback } = {
    ...DEFAULT_NUMBER_OPTIONS,
    ...options,
  };
  if (value === undefined || value === null || Number.isNaN(value)) {
    return fallback;
  }
  return value.toFixed(fractionDigits);
}

/**
 * Formats a Celsius temperature so the readout also exposes the Fahrenheit
 * equivalent. The cockpit dashboard frequently presents sensor data as
 * side-by-side tables, so the helper keeps the output short and symmetrical.
 *
 * @example
 * ```ts
 * formatTemperaturePair(23.1); // "23.1 °C / 73.6 °F"
 * formatTemperaturePair(undefined); // "—"
 * ```
 */
export function formatTemperaturePair(
  celsius: number | null | undefined,
  options: FormatTemperaturePairOptions = {},
): string {
  const {
    celsiusDigits = 1,
    fahrenheitDigits = 1,
    fallback = "—",
  } = options;
  if (celsius === undefined || celsius === null || !Number.isFinite(celsius)) {
    return fallback;
  }
  const fahrenheit = (celsius * 9) / 5 + 32;
  return `${celsius.toFixed(celsiusDigits)} °C / ${
    fahrenheit.toFixed(fahrenheitDigits)
  } °F`;
}

export interface FormatRelativeTimeOptions {
  /**
   * Timestamp in milliseconds used as the reference for the relative
   * calculation. Defaults to `Date.now()` but can be injected for deterministic
   * testing.
   */
  now?: number;
  /** Placeholder used when the timestamp is missing. */
  fallback?: string;
}

const DEFAULT_RELATIVE_OPTIONS: Required<
  Pick<FormatRelativeTimeOptions, "fallback">
> = {
  fallback: "—",
};

/**
 * Formats an absolute timestamp expressed as milliseconds since the UNIX epoch
 * into a terse relative string (e.g. `"3m"`, `"2h"`, `"<1s"`).
 *
 * @example
 * ```ts
 * formatRelativeTime(Date.now() - 1_200); // "1s"
 * formatRelativeTime(undefined); // "—"
 * ```
 */
export function formatRelativeTime(
  timestamp: number | null | undefined,
  options: FormatRelativeTimeOptions = {},
): string {
  const { now = Date.now(), fallback } = {
    ...DEFAULT_RELATIVE_OPTIONS,
    ...options,
  };
  if (timestamp === undefined || timestamp === null) {
    return fallback;
  }
  const delta = now - timestamp;
  if (!Number.isFinite(delta)) {
    return fallback;
  }
  if (delta < 1_000) {
    return "<1s";
  }
  const seconds = Math.floor(delta / 1_000);
  if (seconds < 60) {
    return `${seconds}s`;
  }
  const minutes = Math.floor(seconds / 60);
  if (minutes < 60) {
    return `${minutes}m`;
  }
  const hours = Math.floor(minutes / 60);
  if (hours < 24) {
    return `${hours}h`;
  }
  const days = Math.floor(hours / 24);
  return `${days}d`;
}

export interface FormatBytesOptions {
  /** Placeholder when the payload is not a finite, positive number. */
  fallback?: string;
}

const IEC_UNITS: Array<[number, string]> = [
  [1024 ** 2, "MiB"],
  [1024, "KiB"],
];

/**
 * Formats payload sizes using IEC prefixes (KiB, MiB, …) with a focus on
 * keeping the output compact enough for dashboard tiles.
 */
export function formatBytes(
  bytes: number | null | undefined,
  options: FormatBytesOptions = {},
): string {
  const { fallback = "—" } = options;
  if (
    bytes === undefined || bytes === null || !Number.isFinite(bytes) ||
    bytes <= 0
  ) {
    return fallback;
  }
  for (const [threshold, unit] of IEC_UNITS) {
    if (bytes >= threshold) {
      return `${(bytes / threshold).toFixed(1)} ${unit}`;
    }
  }
  return `${Math.round(bytes)} B`;
}
