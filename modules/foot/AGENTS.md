# Foot module guidelines

- We no longer ship Rust ROS nodes in this module, so avoid introducing `rosidl_generator_rs`, `rclrs`, or other Rust-specific ROS build dependencies when updating patches or manifests.
- Add new patches to `modules/foot/patches/` and rely on `apply_patches.sh` to pick them up automatically; keep patches as small and well-documented as possible.
- The launch script is expected to manage the create_bringup process via traps so `psh` can track and cleanly stop it—preserve the background-launch + `wait` structure when making changes.
- After editing cockpit components, run `node --test modules/foot/cockpit/components/foot-dashboard.helpers.test.js` to keep the helper suite green.
- Keep the cockpit stream action manifest (`cockpit/api/actions.json`) and the mapping helpers in `cockpit/utils/streams.js` aligned whenever telemetry sockets change.
