/**
 * Storage key used to persist cockpit overlay ordering in the dashboard.
 *
 * @example
 * ```ts
 * localStorage.getItem(OVERLAY_ORDER_STORAGE_KEY);
 * ```
 */
export const OVERLAY_ORDER_STORAGE_KEY = "pilot.overlay-grid.order";

/**
 * Parse the persisted overlay order from local storage.
 *
 * @example
 * ```ts
 * const order = parseOverlayOrderJson('["module-a","module-b"]');
 * // order -> ["module-a", "module-b"]
 * ```
 */
export function parseOverlayOrderJson(raw: string | null): string[] {
  if (!raw) return [];
  try {
    const parsed = JSON.parse(raw);
    if (!Array.isArray(parsed)) return [];
    return parsed.filter((value): value is string => typeof value === "string" && value.length > 0);
  } catch (_) {
    return [];
  }
}

/**
 * Reconcile the stored overlay order against the overlays that are currently
 * available.
 *
 * Stored keys that do not match a rendered overlay are ignored and newly
 * rendered overlays are appended to the end of the order.
 *
 * @example
 * ```ts
 * const order = reconcileOverlayOrder(["a", "b", "c"], ["c", "z", "a"]);
 * // order -> ["c", "a", "b"]
 * ```
 */
export function reconcileOverlayOrder(
  availableKeys: readonly string[],
  storedKeys: readonly string[],
): string[] {
  const seen = new Set<string>();
  const ordered: string[] = [];

  for (const key of storedKeys) {
    if (!availableKeys.includes(key)) continue;
    if (seen.has(key)) continue;
    ordered.push(key);
    seen.add(key);
  }

  for (const key of availableKeys) {
    if (seen.has(key)) continue;
    ordered.push(key);
  }

  return ordered;
}

/**
 * Serialise an overlay order for persistence.
 *
 * @example
 * ```ts
 * const payload = serialiseOverlayOrder(["module-a", "module-b"]);
 * // payload -> '["module-a","module-b"]'
 * ```
 */
export function serialiseOverlayOrder(order: readonly string[]): string {
  return JSON.stringify(order);
}
