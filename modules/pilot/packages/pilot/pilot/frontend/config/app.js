/**
 * Minimal controller for the module configuration console.
 *
 * The page fetches module descriptors from the new `/api/module-config` endpoint
 * and renders an editable JSON block for each module. Submissions are persisted
 * immediately through a ``PUT`` request back to the same endpoint.
 */

const moduleState = new Map();
const root = document.getElementById('config-app');
const statusNode = document.querySelector('.config-status');
const refreshButton = document.querySelector('[data-role="refresh"]');

if (!root) {
  throw new Error('Configuration root element not found');
}

if (refreshButton) {
  refreshButton.addEventListener('click', () => loadModules({ announce: true }));
}

loadModules();

/**
 * Fetch the latest module configuration payload and refresh the UI.
 *
 * @param {{ announce?: boolean }} [options] Optional behaviour flags.
 */
async function loadModules(options = {}) {
  setGlobalStatus('Loading module configuration…', 'info');
  try {
    const response = await fetch('/api/module-config');
    if (!response.ok) {
      throw new Error(`Request failed with status ${response.status}`);
    }
    const payload = await response.json();
    const modules = Array.isArray(payload.modules) ? payload.modules : [];
    moduleState.clear();
    modules.forEach((module) => {
      moduleState.set(module.name, {
        original: module.config || {},
        metadata: module,
      });
    });
    renderModules(modules);
    if (options.announce) {
      setGlobalStatus('Refreshed host configuration.', 'success');
    } else {
      clearGlobalStatus();
    }
  } catch (error) {
    console.error('Failed to load module configuration', error);
    setGlobalStatus(
      error instanceof Error ? error.message : 'Failed to load module configuration.',
      'error',
    );
  }
}

/**
 * Render the editor surface for each configured module.
 *
 * @param {Array<Record<string, any>>} modules Resolved module payloads.
 */
function renderModules(modules) {
  root.innerHTML = '';
  if (!modules.length) {
    root.appendChild(emptyState());
    return;
  }

  for (const module of modules) {
    root.appendChild(renderModuleCard(module));
  }
}

/**
 * Return a card element allowing a specific module to be edited.
 *
 * @param {Record<string, any>} module Module payload emitted by the API.
 * @returns {HTMLElement}
 */
function renderModuleCard(module) {
  const card = document.createElement('section');
  card.className = 'config-card';
  card.dataset.moduleName = module.name;

  const header = buildCardHeader(module);

  const editor = document.createElement('textarea');
  editor.className = 'config-card__editor';
  editor.autocomplete = 'off';
  editor.spellcheck = false;
  editor.value = formatConfig(module.config || {});
  editor.setAttribute('aria-label', `${module.display_name || module.name} configuration`);

  const controls = document.createElement('div');
  controls.className = 'config-card__controls';

  const saveButton = document.createElement('button');
  saveButton.type = 'button';
  saveButton.className = 'config-card__save';
  saveButton.textContent = 'Save changes';
  saveButton.addEventListener('click', () => submitModule(card, editor));

  const resetButton = document.createElement('button');
  resetButton.type = 'button';
  resetButton.className = 'config-card__reset';
  resetButton.textContent = 'Reset';
  resetButton.addEventListener('click', () => resetEditor(module.name, editor));

  const status = document.createElement('p');
  status.className = 'config-card__status';
  status.setAttribute('role', 'status');
  status.setAttribute('aria-live', 'polite');

  controls.appendChild(saveButton);
  controls.appendChild(resetButton);

  card.appendChild(header);
  card.appendChild(editor);
  card.appendChild(controls);
  card.appendChild(status);

  return card;
}

/**
 * Submit the updated configuration for a module back to the server.
 *
 * @param {HTMLElement} card Card wrapper for the module being edited.
 * @param {HTMLTextAreaElement} editor Editor containing the JSON payload.
 */
async function submitModule(card, editor) {
  const name = card.dataset.moduleName;
  if (!name) {
    return;
  }

  const status = card.querySelector('.config-card__status');
  const body = editor.value.trim();

  let parsed;
  try {
    parsed = body ? JSON.parse(body) : {};
  } catch (error) {
    setCardStatus(status, 'Configuration must be valid JSON.', 'error');
    editor.focus();
    return;
  }

  setCardStatus(status, 'Saving…', 'info');
  try {
    const response = await fetch(`/api/module-config/${encodeURIComponent(name)}`, {
      method: 'PUT',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(parsed),
    });
    if (!response.ok) {
      const message = await extractError(response);
      throw new Error(message);
    }
    const payload = await response.json();
    moduleState.set(name, {
      original: payload.config || {},
      metadata: payload,
    });
    refreshCardMetadata(card, payload);
    editor.value = formatConfig(payload.config || {});
    setCardStatus(status, 'Saved.', 'success');
    setGlobalStatus(`Updated ${payload.display_name || name}.`, 'success');
  } catch (error) {
    console.error(`Failed to persist configuration for ${name}`, error);
    setCardStatus(
      status,
      error instanceof Error ? error.message : 'Failed to persist configuration.',
      'error',
    );
    setGlobalStatus(
      error instanceof Error ? error.message : 'Failed to persist configuration.',
      'error',
    );
  }
}

/**
 * Reset an editor to the last known configuration fetched from the server.
 *
 * @param {string} moduleName Target module identifier.
 * @param {HTMLTextAreaElement} editor Editor element to update.
 */
function resetEditor(moduleName, editor) {
  const record = moduleState.get(moduleName);
  if (!record) {
    editor.value = '{}';
    return;
  }
  editor.value = formatConfig(record.original || {});
  setGlobalStatus(`Reverted ${record.metadata.display_name || moduleName}.`, 'info');
}

/**
 * Display a shared status message for the whole page.
 *
 * @param {string} message Status text to surface.
 * @param {'info' | 'success' | 'error'} tone Semantic tone for styling.
 */
function setGlobalStatus(message, tone) {
  if (!statusNode) {
    return;
  }
  statusNode.textContent = message;
  statusNode.dataset.tone = tone;
}

/** Clear the shared status banner. */
function clearGlobalStatus() {
  if (!statusNode) {
    return;
  }
  statusNode.textContent = '';
  delete statusNode.dataset.tone;
}

/**
 * Update the inline status element for a module card.
 *
 * @param {Element | null} node Target status node.
 * @param {string} message Status message to present.
 * @param {'info' | 'success' | 'error'} tone Semantic tone for styling.
 */
function setCardStatus(node, message, tone) {
  if (!node) {
    return;
  }
  node.textContent = message;
  node.dataset.tone = tone;
}

/**
 * Render a short “empty state” description when no modules are configured.
 *
 * @returns {HTMLElement}
 */
function emptyState() {
  const wrapper = document.createElement('section');
  wrapper.className = 'config-empty';
  wrapper.innerHTML = `
    <h2>No modules configured</h2>
    <p>The host manifest does not list any modules yet. Add entries under <code>host.modules</code> to begin configuring them.</p>
  `;
  return wrapper;
}

/**
 * Produce a badge element used to highlight module metadata.
 *
 * @param {string} label Text to display inside the badge.
 * @param {'success' | 'link'} [variant] Visual variant for styling.
 * @returns {HTMLAnchorElement | HTMLSpanElement}
 */
function createBadge(label, variant) {
  const element = variant === 'link' ? document.createElement('a') : document.createElement('span');
  element.className = `config-badge${variant ? ` config-badge--${variant}` : ''}`;
  element.textContent = label;
  return element;
}

/**
 * Build a module card header containing the title, badges, and description.
 *
 * @param {Record<string, any>} module Module metadata payload.
 * @returns {HTMLElement}
 */
function buildCardHeader(module) {
  const header = document.createElement('header');
  header.className = 'config-card__header';

  const title = document.createElement('h2');
  title.textContent = module.display_name || module.name;
  header.appendChild(title);

  const badges = document.createElement('div');
  badges.className = 'config-card__badges';

  if (module.listed) {
    badges.appendChild(createBadge('Listed on host'));
  }
  if (module.active) {
    badges.appendChild(createBadge('Launch enabled', 'success'));
  }
  if (module.has_pilot) {
    const link = module.dashboard_url || `/modules/${module.name}/`;
    const badge = createBadge('Pilot dashboard', 'link');
    badge.href = link;
    badge.target = '_blank';
    badge.rel = 'noopener';
    badges.appendChild(badge);
  }

  if (badges.childNodes.length) {
    header.appendChild(badges);
  }

  if (module.description) {
    const description = document.createElement('p');
    description.className = 'config-card__description';
    description.textContent = module.description;
    header.appendChild(description);
  }

  return header;
}

/**
 * Refresh the visible metadata for a module card after a save.
 *
 * @param {HTMLElement} card Card element being updated.
 * @param {Record<string, any>} module Latest module payload from the server.
 */
function refreshCardMetadata(card, module) {
  card.dataset.moduleName = module.name;
  const existing = card.querySelector('.config-card__header');
  const updated = buildCardHeader(module);
  if (existing && existing.parentNode === card) {
    card.replaceChild(updated, existing);
  } else {
    card.insertBefore(updated, card.firstChild);
  }
}

/**
 * Format a configuration object into a JSON string.
 *
 * @param {Record<string, any>} value Arbitrary JSON-compatible value.
 * @returns {string}
 */
function formatConfig(value) {
  try {
    return JSON.stringify(value && typeof value === 'object' ? value : {}, null, 2);
  } catch (_error) {
    return '{}';
  }
}

/**
 * Extract an error message from a failed ``fetch`` response.
 *
 * @param {Response} response Fetch response that failed.
 * @returns {Promise<string>}
 */
async function extractError(response) {
  try {
    const payload = await response.json();
    if (payload && typeof payload === 'object' && typeof payload.message === 'string') {
      return payload.message;
    }
  } catch (_error) {
    // Fall back to response text when JSON parsing fails.
  }
  try {
    const text = await response.text();
    return text || `Request failed with status ${response.status}`;
  } catch (_error) {
    return `Request failed with status ${response.status}`;
  }
}
