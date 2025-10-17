/**
 * Simple runtime registry for sharing helpers between cockpit frontend components.
 *
 * Modules may register a named export using `register(name, value)` and other
 * components can retrieve it via `get(name)`. A convenience `exportsify` helper
 * will register a module's default export under the provided name when present.
 */
const globalKey = '__psyched_frontend_registry__';
if (!window[globalKey]) {
    window[globalKey] = { map: new Map() };
}
const store = window[globalKey];

export function register(name, value) {
    if (typeof name !== 'string' || !name) {
        throw new TypeError('register: name must be a non-empty string');
    }
    store.map.set(name, value);
}

export function get(name) {
    if (typeof name !== 'string' || !name) {
        return undefined;
    }
    return store.map.get(name);
}

export function list() {
    return Array.from(store.map.keys());
}

export function exportsify(name, moduleDefault) {
    if (!moduleDefault) return moduleDefault;
    try {
        register(name, moduleDefault);
    } catch (err) {
        // swallow to avoid breaking consumers if registration fails
        console.warn('exportsify failed', err);
    }
    return moduleDefault;
}

export default { register, get, list, exportsify };
