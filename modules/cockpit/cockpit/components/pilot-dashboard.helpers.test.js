// Minimal test shim for packaging
import { formatPilotStatus } from './pilot-dashboard.helpers.js';
console.assert(formatPilotStatus('ok') === 'ok');
