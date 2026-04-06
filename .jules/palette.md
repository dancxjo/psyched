
## 2024-05-24 - Dynamic Status ARIA Live Regions
**Learning:** In the Cockpit UI, dynamic text updates that use `.surface-status` to convey loading, completion, or error states must explicitly set `role` and `aria-live` attributes. Otherwise, screen readers will not announce changes (e.g., when a host action starts or finishes).
**Action:** Always append `role="status" aria-live="polite"` for non-critical updates and `role="alert" aria-live="assertive"` for errors when rendering `.surface-status` messages.
