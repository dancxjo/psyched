# GPS module guidelines

- After updating cockpit helpers or dashboard components, run
  `deno test modules/gps/cockpit/components/gps-dashboard.helpers.test.js` when
  Deno is available to keep telemetry formatting in sync.
- Run `PYTHONPATH=. pytest modules/gps/pilot/tests` when modifying the pilot
  prompt translators or topic suggestions.
- Run `PYTHONPATH=. pytest tests/gps` after changing the provisioning scripts so
  the gpsd configuration checks stay aligned with the u-blox 7 setup.
