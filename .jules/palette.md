## 2026-04-14 - Use shared surface-button for consistent interactions
**Learning:** Found an error state 'Retry' button in `cockpit-app.js` that was implemented as a generic button without accessible labels or design system classes, leaving it inaccessible and visually inconsistent.
**Action:** Replaced raw buttons with the `surface-button` design token to ensure proper hover, focus, and aria attributes, bringing them in line with the established UI patterns.
