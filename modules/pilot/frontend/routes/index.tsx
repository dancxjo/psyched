import {
  moduleTilesForHost,
  serviceTilesForHost,
} from "../lib/dashboard/tiles.ts";
import { define } from "../utils.ts";

export default define.page(() => {
  const moduleTiles = moduleTilesForHost();
  const serviceTiles = serviceTilesForHost();
  const overlays = [
    ...moduleTiles.map((tile) => ({ ...tile, key: `module-${tile.name}` })),
    ...serviceTiles.map((tile) => ({ ...tile, key: `service-${tile.name}` })),
  ];

  return (
    <section class="content">
      <div class="overlay-grid">
        {overlays.length > 0
          ? overlays.map((tile) => {
            const Overlay = tile.overlay;
            return (
              <div
                key={tile.key}
                class="overlay-grid__item"
                data-kind={tile.kind}
                data-name={tile.name}
              >
                <Overlay {...(tile.overlayProps ?? {})} />
              </div>
            );
          })
          : (
            <p class="overlay-grid__empty">
              No cockpit overlays are enabled for this host.
            </p>
          )}
      </div>
    </section>
  );
});
