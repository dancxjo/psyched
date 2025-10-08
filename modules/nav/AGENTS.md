# Nav module guidelines

- ROS setup scripts frequently reference optional variables such as `AMENT_TRACE_SETUP_FILES`.
  Always guard these sources by temporarily disabling `set -u` or by defining a default before sourcing.
- Prefer wrapping setup sourcing in helpers (see `scripts/build_psyched_nav.sh`) so future scripts reuse the same behaviour.
