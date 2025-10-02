# Vendored ROS 2 Rust message crates

The provisioning flow generates Rust bindings for ROS 2 message packages (e.g. `std_msgs`, `sensor_msgs`) using `rosidl_generator_rs` inside a Docker builder. The resulting crates are copied into this directory so Cargo patches resolve without depending on system-wide colcon workspaces.

Run `tools/bootstrap/generate_ros_rust_bindings.sh` whenever you update `ROS_DISTRO` or need to refresh the bindings:

```bash
ROS_DISTRO=${ROS_DISTRO:-kilted} tools/bootstrap/generate_ros_rust_bindings.sh
```

The script overwrites the package folders below with the freshly generated crates. Commit the updated contents when synchronizing with upstream ROS 2 interface changes.
