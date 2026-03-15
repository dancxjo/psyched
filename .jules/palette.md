## 2024-03-15 - [ARIA Labels for Repeated Action Buttons]
**Learning:** When generating multiple instances of identical interactive elements (like "Save" or "Reset" buttons in a list of configuration cards), a screen reader will read them all identically ("Save changes, button") unless specific `aria-label`s are applied to provide context about what is being saved.
**Action:** Always interpolate the context or ID into the `aria-label` when dynamically creating repeated action buttons (e.g. `Save changes to ${module.name}`).
