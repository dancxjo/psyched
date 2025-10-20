import {
  createCollapsedCardManager,
  normaliseCardId,
  normaliseDashboardScope,
} from '/components/dashboard-collapse.js';

const processedCards = new WeakSet();
const managerCache = new Map();

function getManager(scopeId) {
  const key = normaliseDashboardScope(scopeId);
  if (!managerCache.has(key)) {
    managerCache.set(key, createCollapsedCardManager(key));
  }
  return managerCache.get(key);
}

function resolveScope(card) {
  if (!card) {
    return 'cockpit';
  }
  if (card.dataset && typeof card.dataset.dashboardScope === 'string' && card.dataset.dashboardScope.trim()) {
    return card.dataset.dashboardScope;
  }
  const scoped = card.closest('[data-dashboard-id]');
  if (scoped && scoped.dataset && typeof scoped.dataset.dashboardId === 'string' && scoped.dataset.dashboardId.trim()) {
    return scoped.dataset.dashboardId;
  }
  const section = card.closest('section[id]');
  if (section && section.id) {
    return section.id;
  }
  const ancestorWithId = card.closest('[id]');
  if (ancestorWithId && ancestorWithId.id) {
    return ancestorWithId.id;
  }
  return 'cockpit';
}

function pickTitleElement(card) {
  const direct = Array.from(card.children).find((child) => child.classList && child.classList.contains('surface-card__title'));
  if (direct) {
    return direct;
  }
  return card.querySelector('.surface-card__title');
}

function ensureHeader(card) {
  let header = Array.from(card.children).find((child) => child.classList && child.classList.contains('surface-card__header'));
  if (!header) {
    header = document.createElement('div');
    header.className = 'surface-card__header';
    card.insertBefore(header, card.firstChild || null);
  }
  let title = header.querySelector('.surface-card__title');
  if (!title) {
    const existingTitle = pickTitleElement(card);
    if (existingTitle && existingTitle.parentElement !== header) {
      header.insertBefore(existingTitle, header.firstChild || null);
      title = existingTitle;
    } else {
      title = document.createElement('h3');
      title.className = 'surface-card__title';
      if (existingTitle) {
        title.textContent = existingTitle.textContent || '';
        existingTitle.remove();
      }
      header.insertBefore(title, header.firstChild || null);
    }
  }
  return { header, title };
}

function ensureContent(card, header) {
  let content = Array.from(card.children).find((child) => child.classList && child.classList.contains('surface-card__content'));
  if (!content) {
    content = document.createElement('div');
    content.className = 'surface-card__content';
    while (header.nextSibling) {
      content.appendChild(header.nextSibling);
    }
    card.appendChild(content);
  }
  return content;
}

function resolveCardId(card, titleEl) {
  if (!card) {
    return '';
  }
  const explicit = card.dataset && typeof card.dataset.cardId === 'string' ? normaliseCardId(card.dataset.cardId) : '';
  if (explicit) {
    card.dataset.cardId = explicit;
    return explicit;
  }
  const fromTitle = titleEl && typeof titleEl.textContent === 'string' ? normaliseCardId(titleEl.textContent) : '';
  if (fromTitle) {
    card.dataset.cardId = fromTitle;
    return fromTitle;
  }
  const fallback = normaliseCardId(card.id || '');
  if (fallback) {
    card.dataset.cardId = fallback;
    return fallback;
  }
  return '';
}

function ensureToggle(header) {
  let toggle = header.querySelector('.surface-card__toggle');
  if (!toggle) {
    toggle = document.createElement('button');
    toggle.type = 'button';
    toggle.className = 'surface-card__toggle';
    header.appendChild(toggle);
  }
  return toggle;
}

function applyCollapsedState(card, toggle, content, collapsed, titleText) {
  const label = titleText ? titleText.trim() : 'section';
  card.classList.toggle('surface-card--collapsible', true);
  card.classList.toggle('surface-card--collapsed', Boolean(collapsed));
  if (content) {
    content.hidden = Boolean(collapsed);
  }
  toggle.textContent = collapsed ? 'Expand' : 'Collapse';
  toggle.setAttribute('aria-expanded', collapsed ? 'false' : 'true');
  toggle.setAttribute('aria-label', collapsed ? `Expand ${label}` : `Collapse ${label}`);
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
  const contentId = cardId ? `${cardId}-content` : '';
  if (contentId) {
    content.id = contentId;
    toggle.setAttribute('aria-controls', contentId);
  }

  let collapsed = cardId ? manager.isCollapsed(cardId) : card.classList.contains('surface-card--collapsed');
  applyCollapsedState(card, toggle, content, collapsed, title ? title.textContent : 'section');

  toggle.addEventListener('click', () => {
    if (cardId) {
      const nextSet = manager.toggle(cardId);
      collapsed = nextSet.has(cardId);
    } else {
      collapsed = !collapsed;
    }
    applyCollapsedState(card, toggle, content, collapsed, title ? title.textContent : 'section');
  });
}

function scanForCards(root = document) {
  if (!root) {
    return;
  }
  const cards = root.querySelectorAll ? root.querySelectorAll('.surface-card') : [];
  for (const card of cards) {
    initialiseCard(card);
  }
  if (root instanceof HTMLElement && root.classList && root.classList.contains('surface-card')) {
    initialiseCard(root);
  }
}

if (typeof window !== 'undefined') {
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', () => scanForCards(document));
  } else {
    scanForCards(document);
  }

  const observer = new MutationObserver((mutations) => {
    for (const mutation of mutations) {
      for (const node of mutation.addedNodes) {
        if (node instanceof HTMLElement) {
          scanForCards(node);
        }
      }
    }
  });

  if (document.body) {
    observer.observe(document.body, { childList: true, subtree: true });
  }
}
