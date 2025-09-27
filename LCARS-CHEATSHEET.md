# LCARS Cheatsheet + Agent Meta-Instructions (v24.2+)

This single-file bundle is designed to be fed to another LLM agent (or a human) so it knows how to render LCARS elements and apply the Pilot UI joystick / D-Pad console improvements used in this repository.

Include this file in the workspace and use it as the canonical reference when editing `modules/pilot/packages/pilot/pilot/static/*` files.

---

## Attribution (required)

LCARS Inspired Website Template by http://www.TheLCARS.com

Keep this attribution somewhere in any public or redistributed HTML that uses the LCARS template.

---

## Quick Reference — LCARS v24.2+ (Condensed)

1. Lists

- Use `<ul class="lcars-list">` with `<li>` elements.
- Bullet color utility classes: `bullet-<color>` (e.g. `bullet-mars`, `bullet-gold`).
- Pre-v24: `oc-<color>` (older class names).

2. Horizontal Bars

- v24+: `<hr class="lcars-bar">`
- Pre-v23-B fallback: `<div class="lcars-bar"><div class="lcars-bar-inner"></div></div>`

3. Blink Effects

- Add `blink`, `blink-slow`, or `blink-fast` to any element.
- Combine with `font-<color>` (e.g. `blink-fast font-mars`) for colored blinking text.

4. Text Utilities

- `flush` = remove top margin after a header
- `nomar` = reset margins
- Alignment: `go-left`, `go-right`, `go-center`
- Size/transform: `go-big` (bigger text)
- `uppercase`, `strike`, `now` (no wrap), `big-sky` (extra top margin)
- Colors: v24+: `font-<color>` (preferred). Pre-v24: `go-<color>`.

5. Sidebar Buttons

- Typical structure (v24.2+):

```html
<div class="sidebar-nav">
  <button onclick="playSoundAndRedirect('audio2','#')">label</button>
</div>
```

- Use `<a>` for silent navigation (no click sound).
- Color helpers: `button-<color>` (e.g. `button-gold`).

6. Panel Buttons

- Replace `panel-n` with a `panel-button` when you want a clickable panel:

```html
<button onclick="playSoundAndRedirect('audio2','#')" class="panel-button pan-x">text</button>
```

- `x` is panel number (e.g. `pan-5`). Use `panel-0` for an extra panel.

7. Text Bar

- Use `.lcars-text-bar`.

```html
<div class="lcars-text-bar"><span>hello world</span></div>
```

- Align at end: add `.the-end`. Color: `font-<color>`.

8. Content Buttons

- Standard:

```html
<div class="buttons">
  <a href="#" class="button-gold">engage</a>
</div>
```

- Blink works on buttons (`class="button-gold blink"`).
- Layout helpers: `justify-center`, `justify-flex-end`, `justify-space-evenly`, `justify-space-between`.

---

## Custom Console Extensions (Pilot UI)

These are the custom components used by the Pilot UI. Prefer 2D SVG + LCARS classes over external libs.

### Joystick Panel (semantic IDs & expected behavior)

- Purpose: show commanded translation/rotation and measured IMU/accel data.
- Core DOM structure (recommended):

```html
<div class="panel panel-joystick" id="joystickPanel">
  <h3 class="flush font-gold">Joystick</h3>

  <div id="joystick" class="joystick" aria-label="joystick control" role="application">
    <!-- SVG knob that the controller manipulates -->
    <svg id="joystickSvg" viewBox="-50 -50 100 100" class="joystick-svg">
      <circle cx="0" cy="0" r="48" class="joystick-ring"/>
      <g id="robotIcon" transform="rotate(0)">
        <!-- simple robot path or icon -->
        <rect x="-6" y="-8" width="12" height="16" class="robot-body"/>
      </g>

      <!-- commanded (blue) knob -->
      <circle id="joystickKnob" class="joystick-knob" cx="0" cy="0" r="10" />

      <!-- measured accel (red) -->
      <circle id="joystickMeasured" class="joystick-measured" cx="0" cy="0" r="4" />

      <!-- yaw arc (orange) -->
      <path id="joystickYawArc" class="joystick-yaw-arc" d="" />
    </svg>

    <div class="joystick-meta">
      <span id="cmdVelTopic" class="font-sky">CmdVel:</span>
      <span id="speedValue">0.00</span>
    </div>
  </div>
</div>
```

- Required IDs (do not break): `joystick`, `joystickSvg`, `joystickKnob`, `joystickMeasured`, `joystickYawArc`, `robotIcon`, `cmdVelTopic`, `speedValue`.
- State attributes JS will set: `data-dragging`, `data-enabled`, and `data-status`.

Notes:
- Keep geometry updates (knob transform, battery fill, IMU stroke math) as numeric writes when necessary; prefer to expose them through CSS custom properties (`--knob-x`, `--knob-y`) if doing a full separation-of-concerns refactor.
- Use `transform` only for knob movement (GPU accelerated).

### D-Pad Panel

- Recommended markup (LCARS classes can be added):

```html
<div class="panel panel-dpad" id="dpad">
  <h3 class="flush font-gold">D-Pad</h3>
  <div class="dpad" role="group" aria-label="directional pad">
    <button id="dpadUp" class="dpad-btn dpad-up" aria-label="up">▲</button>
    <div class="dpad-middle">
      <button id="dpadLeft" class="dpad-btn dpad-left">◀</button>
      <button id="dpadCenter" class="dpad-btn dpad-center">●</button>
      <button id="dpadRight" class="dpad-btn dpad-right">▶</button>
    </div>
    <button id="dpadDown" class="dpad-btn dpad-down">▼</button>
  </div>
</div>
```

- Required IDs: `dpadUp`, `dpadDown`, `dpadLeft`, `dpadRight`, `dpadCenter`.
- D-pad press behavior: long-press (repeat) and single-press. The existing `joystick.js` contains logic for repeat intervals; prefer to reuse it.

### Attitude / Horizon

- Small canvas or SVG showing pitch/roll. Use `id="horizonCanvas"` or `id="horizonSvg"`.
- Optional cube icon; ensure its transforms are updated by JS.

### Telemetry Panel

- Layout:

```html
<div class="panel panel-telemetry">
  <h3 class="flush font-gold">Telemetry</h3>
  <ul class="lcars-list telemetry-list">
    <li class="bullet-mars">Speed: <span id="tele-speed">0.0 m/s</span></li>
    <li class="bullet-gold">Turn: <span id="tele-turn">0.0 rad/s</span></li>
    <li class="bullet-sky">Pitch: <span id="tele-pitch">0.0°</span></li>
  </ul>
</div>
```

---

## Data-Attribute Conventions (for JS ⇄ CSS separation)

Use these attributes instead of adding/removing presentation classes from JS. CSS should style the visual state keyed to these attributes.

- `data-dragging="true"` on `#joystick` when knob is being dragged.
- `data-enabled="false"|"true"` for any control that can be toggled.
- `data-visible="false"|"true"` (for overlays such as depth image).
- `data-opacity="0.5"` for numeric opacity (string form). Note: `attr()` in CSS is limited; prefer CSS variables when needing numeric values in `calc()`.
- `data-status="ok|warn|error"` on a status container to drive color/emoji.
- `data-error="true"` on a service pill to show an outlined error state.

CSS examples should use attribute selectors, e.g. `#joystick[data-dragging="true"] .joystick-knob { box-shadow: 0 6px 12px rgba(0,0,0,.35); }`.

---

## Minimal CSS Snippets (examples)

These are examples that pair with the LCARS template. Import `assets/lower-decks-padd.css` first (the canonical LCARS). Keep these rules in `style.css`.

```css
/* structural joystick rules */
.joystick { width: 240px; height: 240px; display: inline-block; }
.joystick-svg { width: 100%; height: 100%; }
.joystick-knob { fill: var(--lcars-blue, #4aa3ff); transition: transform 80ms linear; }
.joystick-measured { fill: var(--lcars-red, #ff4d4f); opacity: 0.95; }
.joystick-yaw-arc { stroke: var(--lcars-orange, #ff9900); stroke-width: 3; fill: none; }
#joystick[data-dragging="true"] .joystick-knob { filter: drop-shadow(0 8px 10px rgba(0,0,0,0.45)); }

/* dpad */
.dpad { display:flex; flex-direction:column; align-items:center; }
.dpad-middle{ display:flex; gap:8px; }
.dpad-btn { background:var(--panel-bg, #111); border-radius:6px; padding:10px; min-width:36px; }
.dpad-btn:active{ transform:translateY(1px); }

/* telemetry list bullets */
.lcars-list .bullet-mars::marker { color: var(--lcars-mars, #ff7a7a); }

/* status */
.status[data-status="ok"]{ color:var(--font-sky); }
.status[data-status="error"]{ color:var(--font-mars); }

/* service pill error */
.pill[data-error="true"] { outline: 2px solid #ef4444; }
```

Note: Prefer LCARS color helper classes (e.g. `font-mars`, `button-gold`) for consistent colors.

---

## Small JS Helper Patterns (conservative safe examples)

Keep presentation toggles as attributes and numeric geometry as either inline transforms or CSS variables.

1) Toggle attribute helper

```js
function setAttr(el, name, value) {
  if (value === false || value === null || value === undefined) el.removeAttribute(name);
  else el.setAttribute(name, String(value));
}

// Usage:
setAttr(document.getElementById('joystick'), 'data-dragging', true);
```

2) Optionally expose numeric knob position via CSS vars (safer than many inline styles):

```js
function setKnobPosition(elSvg, xPx, yPx) {
  // e.g. set CSS vars used in :root or the element
  elSvg.style.setProperty('--knob-x', xPx + 'px');
  elSvg.style.setProperty('--knob-y', yPx + 'px');
  // still OK to use transform for immediate GPU rendering
  const knob = elSvg.querySelector('#joystickKnob');
  knob.style.transform = `translate(${xPx}px, ${yPx}px)`;
}
```

3) Battery / IMU numeric visuals

- Battery fill: keep `el.style.width = `${percent}%`` or expose `--battery-percent` CSS variable and let CSS `width: var(--battery-percent)`.
- IMU arcs: keep `stroke-dasharray` / `stroke-dashoffset` updates (SVG maths) — or compute and write into CSS custom properties and use `stroke-dashoffset: var(--dash-offset)`.

---

## Integration Rules (for an LLM agent editing files)

When editing `modules/pilot/packages/pilot/pilot/static/index.html` or any static assets:

- Preserve the exact IDs and ARIA attributes used by `joystick.js`. See "Required IDs" above.
- Never remove or rename: `joystick`, `joystickKnob`, `joystickMeasured`, `joystickYawArc`, `robotIcon`, `dpadUp`, `dpadDown`, `dpadLeft`, `dpadRight`, `dpadCenter`, `batteryFill`, `batteryPercentInfo`, `depth-image`, `horizonCanvas` (or `horizonSvg`), and any `id` values referenced by JS.
- Prefer adding LCARS classes (`panel-`, `font-`, `button-`, `bullet-`) rather than inline styles or hex colors.
- For stateful presentation, set `data-*` attributes instead of toggling classes in JS. If you must change class names, ensure the CSS mapping is added to `style.css` and does not conflict with LCARS assets.
- Add the required attribution line where the LCARS template is used.

---

## Edge Cases & Guidance

1. CSS `attr()` limitations

- CSS `attr()` only returns strings and is limited in where it can be used (not usable inside `calc()` reliably in browsers). If you need numeric values in CSS calculations, use CSS variables (custom properties) instead (set via `element.style.setProperty('--foo', '0.5')`).

2. JS re-introducing styles

- Search for `className =`, `.classList.add`, `.style.` writes.
- If editing JS to remove presentation writes, convert them to `setAttr` calls and ensure corresponding CSS attribute selectors exist.

3. Host-specific copies

- Hosts (e.g., `hosts/cerebellum/...`) may contain duplicate files. Keep the primary workspace `modules/pilot/packages/pilot/pilot/static/` files as the canonical source. Update host copies when deploying or when host-specific overrides are needed.

4. Offline / Network-limited build environments

- LCARS assets should be part of `static/assets/` in the repo. Do not attempt to download assets during editing.

---

## Quality Gates & Tests (quick checklist)

When making changes, an agent should run the following lightweight checks:

1. Lint/Static Checks
- Verify HTML is valid and `id`s referenced by JS still exist.

2. Smoke Test (serve static files locally)

```bash
# from repository root
cd modules/pilot/packages/pilot/pilot/static
python3 -m http.server 8000
# open http://localhost:8000/index.html in a browser
```

3. Console checks
- Open browser console and search for JS errors referencing missing elements (e.g. `document.getElementById(...) returned null`).

4. Interaction checks
- Drag the joystick; ensure `data-dragging` toggles on the `#joystick` element and knob moves.
- Press D-pad buttons (short and long press) and confirm command messages are sent (or logged) by existing websocket code.

5. Unit tests (optional)
- Add small `pytest` tests under `packages/pilot/tests/` stubbing DOM if needed. But starting with smoke tests is sufficient.

Record PASS/FAIL for each gate in a short developer message.

---

## Rollout Steps for an Agent (recommended)

1. Create/update UI in `modules/pilot/packages/pilot/pilot/static/index.html` using LCARS template but keep required IDs.
2. Add/adjust presentation CSS in `modules/pilot/packages/pilot/pilot/static/style.css` (import `assets/lower-decks-padd.css` first).
3. Refactor `joystick.js` presentation toggles to `data-*` attributes where appropriate.
4. Run the smoke server and validate.
5. If host-specific files are used, sync changes into `hosts/<host>/config/pilot.yaml` (or the relevant YAML file).

---

## Example: Minimal Joystick + D-Pad Component (copyable)

```html
<!-- Add inside a panel container in index.html -->
<div class="panel panel-joystick">
  <h3 class="flush font-gold">Joystick</h3>
  <!-- joystick SVG (IDs required) -->
  <div id="joystick" role="application" aria-label="joystick" data-enabled="true">
    <svg id="joystickSvg" viewBox="-50 -50 100 100" class="joystick-svg">
      <circle r="48" class="joystick-ring"/>
      <g id="robotIcon"></g>
      <path id="joystickYawArc" class="joystick-yaw-arc"/>
      <circle id="joystickMeasured" class="joystick-measured" r="4"/>
      <circle id="joystickKnob" class="joystick-knob" r="10"/>
    </svg>
  </div>

  <div class="dpad">
    <button id="dpadUp" class="dpad-btn">▲</button>
    <div class="dpad-middle">
      <button id="dpadLeft" class="dpad-btn">◀</button>
      <button id="dpadCenter" class="dpad-btn">●</button>
      <button id="dpadRight" class="dpad-btn">▶</button>
    </div>
    <button id="dpadDown" class="dpad-btn">▼</button>
  </div>
</div>
```

---

## Final Notes for the Agent

- Prefer small, reversible edits. Keep working copies and use Git commits for each logical change.
- When in doubt about changing numeric inline style writes (e.g. knob transform), leave them and instead expose `data-*` attributes for visual state until you can refactor fully to CSS variables.
- Ask for confirmation only when a change is destructive (renaming required IDs or removing a file). For styling choices, proceed and include a rollback commit.

---

If you'd like, I can now:

- 1) Add this file to the repo (done).
- 2) Apply the example joystick/D-pad HTML into `index.html` (I can do that and preserve the existing IDs/ARIA). Reply `apply` to proceed.
- 3) Refactor the remaining numeric inline-style writes in `joystick.js` into CSS variables (more invasive). Reply `refactor` to proceed.

Which next step do you want me to take?
