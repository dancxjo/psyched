## 2026-04-01 - Dynamic Status Messages ARIA Accessibility
**Learning:** Dynamic status messages rendered by Lit components (e.g., `<p class="surface-status">`) are not automatically announced by screen readers when their content changes unless proper ARIA live region attributes are used.
**Action:** Always add `role="status" aria-live="polite"` to regular dynamic updates and `role="alert" aria-live="assertive"` to error or critical feedback messages in Lit templates to ensure accessibility.
