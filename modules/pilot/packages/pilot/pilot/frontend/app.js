/**
 * Entry point for the Psyched pilot frontend.
 *
 * Registers the custom elements defined in the component modules and mounts
 * the <pilot-app> shell into the page placeholder.
 */

import './components/pilot-app.js';

const mountPoint = document.getElementById('pilot-app');

if (mountPoint && !mountPoint.querySelector('pilot-app')) {
  mountPoint.appendChild(document.createElement('pilot-app'));
}
