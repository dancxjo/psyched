import PilotApp from './components/PilotApp.svelte';

const target = document.getElementById('pilot-app');

if (target) {
  new PilotApp({ target });
}
