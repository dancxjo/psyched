# Hypothalamus module guidelines

- Keep the simulated fallback online so pilot dashboards remain responsive when
  hardware is offline.
- Mirror Celsius readings with Fahrenheit publications and cockpit readouts for
  operator familiarity.
- Prefer dependency injection for hardware adapters to keep unit tests fast and
  deterministic.
