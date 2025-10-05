# Foot module guidelines

- We no longer ship Rust ROS nodes in this module, so avoid introducing `rosidl_generator_rs`, `rclrs`, or other Rust-specific ROS build dependencies when updating patches or manifests.
- Add new patches to `modules/foot/patches/` and rely on `apply_patches.sh` to pick them up automatically; keep patches as small and well-documented as possible.
