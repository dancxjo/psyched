## 2025-04-02 - Add tooltips to icon-only buttons for hover accessibility
**Learning:** I discovered that several icon-only buttons with "aria-label" in the cockpit web UI were missing the "title" attribute. This lack of "title" caused a missing native hover tooltip for users relying on mouse navigation.
**Action:** Ensure all icon-only buttons not only have "aria-label" or "aria-hidden" spans but also have the "title" attribute attached so hover tooltips appear natively.
