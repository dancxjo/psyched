import {
  createCollapsedCardManager,
  normaliseCardId,
  normaliseDashboardScope,
} from "/components/dashboard-collapse.js";

const processedCards = new WeakSet();
const managerCache = new Map();
const observedShadowRoots = new WeakSet();

function getManager(scopeId) {
  const key = normaliseDashboardScope(scopeId);
  if (!managerCache.has(key)) {
    managerCache.set(key, createCollapsedCardManager(key));
  }
  return managerCache.get(key);
}

function resolveScope(card) {
  if (!card) {
    return "cockpit";
  }
  if (
    card.dataset && typeof card.dataset.dashboardScope === "string" &&
    card.dataset.dashboardScope.trim()
  ) {
    return card.dataset.dashboardScope;
  }
  const scoped = card.closest("[data-dashboard-id]");
  if (
    scoped && scoped.dataset &&
    typeof scoped.dataset.dashboardId === "string" &&
    scoped.dataset.dashboardId.trim()
  ) {
    return scoped.dataset.dashboardId;
  }
  const section = card.closest("section[id]");
  if (section && section.id) {
    return section.id;
  }
  const ancestorWithId = card.closest("[id]");
  if (ancestorWithId && ancestorWithId.id) {
    return ancestorWithId.id;
  }
  return "cockpit";
}

function pickTitleElement(card) {
  const direct = Array.from(card.children).find((child) =>
    child.classList && child.classList.contains("surface-card__title")
  );
  if (direct) {
    return direct;
  }
  return card.querySelector(".surface-card__title");
}

function ensureHeader(card) {
  let header = Array.from(card.children).find((child) =>
    child.classList && child.classList.contains("surface-card__header")
  );
  if (!header) {
    header = document.createElement("div");
    header.className = "surface-card__header";
    card.insertBefore(header, card.firstChild || null);
  }
  let title = header.querySelector(".surface-card__title");
  if (!title) {
    const existingTitle = pickTitleElement(card);
    if (existingTitle && existingTitle.parentElement !== header) {
      header.insertBefore(existingTitle, header.firstChild || null);
      title = existingTitle;
    } else {
      title = document.createElement("h3");
      title.className = "surface-card__title";
      if (existingTitle) {
        title.textContent = existingTitle.textContent || "";
        existingTitle.remove();
      }
      header.insertBefore(title, header.firstChild || null);
    }
  }
  return { header, title };
}

function ensureContent(card, header) {
  let content = Array.from(card.children).find((child) =>
    child.classList && child.classList.contains("surface-card__content")
  );
  if (!content) {
    content = document.createElement("div");
    content.className = "surface-card__content";
    while (header.nextSibling) {
      content.appendChild(header.nextSibling);
    }
    card.appendChild(content);
  }
  return content;
}

function resolveCardId(card, titleEl) {
  if (!card) {
    return "";
  }
  const explicit = card.dataset && typeof card.dataset.cardId === "string"
    ? normaliseCardId(card.dataset.cardId)
    : "";
  if (explicit) {
    card.dataset.cardId = explicit;
    return explicit;
  }
  const fromTitle = titleEl && typeof titleEl.textContent === "string"
    ? normaliseCardId(titleEl.textContent)
    : "";
  if (fromTitle) {
    card.dataset.cardId = fromTitle;
    return fromTitle;
  }
  const fallback = normaliseCardId(card.id || "");
  if (fallback) {
    card.dataset.cardId = fallback;
    return fallback;
  }
  return "";
}

function ensureToggle(header) {
  let toggle = header.querySelector(".surface-card__toggle");
  if (!toggle) {
    toggle = document.createElement("button");
    toggle.type = "button";
    toggle.className = "surface-card__toggle";
    header.appendChild(toggle);
  }
  return toggle;
}

function ensureToggleParts(toggle) {
  if (!toggle) {
    return { icon: null, srText: null, label: null };
  }

  if (toggle.dataset.toggleDecorated !== "true") {
    toggle.textContent = "";
    toggle.dataset.toggleDecorated = "true";
  }

  let icon = toggle.querySelector(".surface-card__toggleIcon");
  if (!icon) {
    icon = document.createElement("span");
    icon.className = "surface-card__toggleIcon";
    icon.setAttribute("aria-hidden", "true");
    toggle.appendChild(icon);
  }

  let label = toggle.querySelector(".surface-card__toggleLabel");
  if (!label) {
    label = document.createElement("span");
    label.className = "surface-card__toggleLabel";
    label.setAttribute("aria-hidden", "true");
    toggle.appendChild(label);
  }

  let srText = toggle.querySelector(".surface-card__toggleText");
  if (!srText) {
    srText = document.createElement("span");
    srText.className = "surface-card__toggleText";
    toggle.appendChild(srText);
  }

  return { icon, srText, label };
}

function applyCollapsedState(card, toggle, content, collapsed, titleText) {
  const label = titleText ? titleText.trim() : "section";
  card.classList.toggle("surface-card--collapsible", true);
  card.classList.toggle("surface-card--collapsed", Boolean(collapsed));
  if (content) {
    content.hidden = Boolean(collapsed);
  }
  const { icon, srText, label: visibleLabel } = ensureToggleParts(toggle);
  if (icon) {
    icon.textContent = collapsed ? "▸" : "▾";
  }
  const hiddenLabel = collapsed ? `Expand ${label}` : `Collapse ${label}`;
  if (visibleLabel) {
    visibleLabel.textContent = collapsed ? "Expand" : "Collapse";
  }
  if (srText) {
    srText.textContent = hiddenLabel;
  }
  toggle.setAttribute("aria-expanded", collapsed ? "false" : "true");
  toggle.setAttribute("aria-label", hiddenLabel);
}

function monitorShadowRoot(shadowRoot) {
  if (!shadowRoot || observedShadowRoots.has(shadowRoot)) {
    return;
  }
  observedShadowRoots.add(shadowRoot);
  scanForCards(shadowRoot);

  const observer = new MutationObserver((mutations) => {
    for (const mutation of mutations) {
      for (const node of mutation.addedNodes) {
        handlePotentialRoot(node);
      }
    }
  });
  observer.observe(shadowRoot, { childList: true, subtree: true });
}

function handlePotentialRoot(node) {
  if (!node) {
    return;
  }
  if (node instanceof HTMLElement) {
    scanForCards(node);
    if (node.shadowRoot) {
      monitorShadowRoot(node.shadowRoot);
    }
  } else if (node instanceof ShadowRoot) {
    monitorShadowRoot(node);
  } else if (node instanceof DocumentFragment) {
    scanForCards(node);
    for (
      const element of node.querySelectorAll ? node.querySelectorAll("*") : []
    ) {
      if (element instanceof HTMLElement && element.shadowRoot) {
        monitorShadowRoot(element.shadowRoot);
      }
    }
  }
}

function trackShadowHosts(root) {
  if (!root) {
    return;
  }

  if (root instanceof HTMLElement && root.shadowRoot) {
    monitorShadowRoot(root.shadowRoot);
  }

  const scope = root.querySelectorAll ? root : null;
  if (!scope) {
    return;
  }

  for (const element of scope.querySelectorAll("*")) {
    if (element instanceof HTMLElement && element.shadowRoot) {
      monitorShadowRoot(element.shadowRoot);
    }
  }
}

function initialiseCard(card) {
  if (!(card instanceof HTMLElement)) {
    return;
  }
  if (processedCards.has(card)) {
    return;
  }
  processedCards.add(card);

  const { header, title } = ensureHeader(card);
  const content = ensureContent(card, header);
  const cardId = resolveCardId(card, title);
  const scopeId = resolveScope(card);
  const manager = getManager(scopeId);
  const toggle = ensureToggle(header);
  const contentId = cardId ? `${cardId}-content` : "";
  if (contentId) {
    content.id = contentId;
    toggle.setAttribute("aria-controls", contentId);
  }

  const defaultCollapsed = card.classList.contains("surface-card--collapsed");
  if (cardId && defaultCollapsed && !manager.isDefaultApplied(cardId)) {
    if (!manager.isCollapsed(cardId)) {
      manager.setCollapsed(cardId, true);
    }
    manager.markDefaultApplied(cardId);
  }

  let collapsed = cardId ? manager.isCollapsed(cardId) : defaultCollapsed;
  applyCollapsedState(
    card,
    toggle,
    content,
    collapsed,
    title ? title.textContent : "section",
  );

  toggle.addEventListener("click", () => {
    if (cardId) {
      const nextSet = manager.toggle(cardId);
      collapsed = nextSet.has(cardId);
    } else {
      collapsed = !collapsed;
    }
    applyCollapsedState(
      card,
      toggle,
      content,
      collapsed,
      title ? title.textContent : "section",
    );
  });
}

function scanForCards(root = document) {
  if (!root) {
    return;
  }

  const cards = root.querySelectorAll
    ? root.querySelectorAll(".surface-card")
    : [];
  for (const card of cards) {
    initialiseCard(card);
  }
  if (
    root instanceof HTMLElement && root.classList &&
    root.classList.contains("surface-card")
  ) {
    initialiseCard(root);
  }

  trackShadowHosts(root);
}

if (typeof window !== "undefined") {
  const initialise = () => {
    scanForCards(document);
  };
  if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", initialise);
  } else {
    initialise();
  }

  const observer = new MutationObserver((mutations) => {
    for (const mutation of mutations) {
      for (const node of mutation.addedNodes) {
        handlePotentialRoot(node);
      }
    }
  });

  if (document.body) {
    observer.observe(document.body, { childList: true, subtree: true });
    trackShadowHosts(document.body);
  }
}
