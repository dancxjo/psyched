export function pilotDashboard() {
  return {
    modules: [],
    loading: true,
    error: '',
    async init() {
      try {
        const response = await fetch('/api/modules');
        if (!response.ok) {
          throw new Error(`Request failed with status ${response.status}`);
        }
        const payload = await response.json();
        this.modules = payload.modules ?? [];
      } catch (error) {
        console.error('Failed to load modules', error);
        this.error = error instanceof Error ? error.message : String(error);
      } finally {
        this.loading = false;
      }
    },
    moduleUrl(module) {
      if (module.slug) {
        return `/modules/${module.slug}/index.html`;
      }
      return `/modules/${module.name}/index.html`;
    },
  };
}

export function createTopicSocket({ topic, type, role = 'subscribe' }) {
  if (!topic) throw new Error('topic is required');
  if (!type) throw new Error('type is required');
  const location = new URL(window.location.href);
  location.pathname = '/api/topics/bridge';
  location.searchParams.set('topic', topic);
  location.searchParams.set('type', type);
  location.searchParams.set('role', role);
  if (location.protocol === 'http:') {
    location.protocol = 'ws:';
  } else if (location.protocol === 'https:') {
    location.protocol = 'wss:';
  }
  return new WebSocket(location.toString());
}

if (typeof window !== 'undefined') {
  const pilotGlobals = window.Pilot ? { ...window.Pilot } : {};
  pilotGlobals.createTopicSocket = createTopicSocket;
  pilotGlobals.dashboard = pilotDashboard;
  window.Pilot = pilotGlobals;
  window.pilotDashboard = pilotDashboard;
}
