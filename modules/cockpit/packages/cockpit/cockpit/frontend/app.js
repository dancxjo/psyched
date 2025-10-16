/**
 * Entry point for the Psyched cockpit frontend.
 *
 * Registers the custom elements defined in the component modules and mounts
 * the <cockpit-app> shell into the page placeholder.
 */

import './components/cockpit-app.js';

const mountPoint = document.getElementById('cockpit-app');

if (mountPoint && !mountPoint.querySelector('cockpit-app')) {
  mountPoint.appendChild(document.createElement('cockpit-app'));
}
