## 2024-04-09 - Accessible Dynamic Status Messages

**Learning:** Dynamic status messages indicating loading states, errors, or timestamp updates in the cockpit UI (often using the `.surface-status` class) were frequently inaccessible to screen reader users because they lacked proper ARIA live region declarations. Without these, assistive technologies would not announce state changes unless the user explicitly navigated to them.

**Action:** Consistently apply `role="status"` with `aria-live="polite"` for non-critical dynamic updates (like "Loading..." or "Updated...") and `role="alert"` with `aria-live="assertive"` for critical error or warning states across all dynamic status elements in the cockpit UI.
