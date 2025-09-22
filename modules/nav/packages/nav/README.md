Navigation and 3D SLAM module for Psyched
======================================

This module provides a scaffold to bring up Nav2 and RTAB-Map for 3D mapping
using a depth camera (the Kinect mounted over the Create 1 base). It is a
starting point — you will need to adapt topics, TF frames, and parameters to
match your robot.

Design choices
--------------
- RTAB-Map is chosen for 3D mapping from RGB-D sensors because it provides
  loop closure, global map export, and is widely used with Kinect and RealSense.
- Cartographer is primarily 2D/2.5D and more focused on lidar — it can do 3D
  with many caveats. For Kinect-based dense 3D reconstructions, RTAB-Map is
  generally the pragmatic choice.

Installation (apt packages)
---------------------------
On Ubuntu with ROS2 installed (kilted/humble/foxy/etc.) run:

```bash
sudo apt update
sudo apt install -y ros-${ROS_DISTRO}-nav2-bringup ros-${ROS_DISTRO}-rtabmap-ros \
  ros-${ROS_DISTRO}-octomap-ros ros-${ROS_DISTRO}-pcl-conversions \
  libpcl-dev ros-${ROS_DISTRO}-pcl-ros
```

If your environment blocks network access (see repository root docs), the
setup script will note the failure and suggest manual installation steps.

How the module is organized
--------------------------
- `launch.sh` — wrapper that runs `ros2 launch nav nav_bringup.launch.py`.
- `setup.sh` — (coming) installs apt packages and optionally clones helpers.
- `packages/nav/nav_bringup.launch.py` — minimal launch composition for nav2
  and rtabmap; edit to include your parameter files and frames.

Added files
-----------
- `params/nav2_params.yaml` — minimal placeholder for Nav2 parameters
- `params/rtabmap_params.yaml` — RTAB-Map parameter defaults for Kinect
- `save_map.sh` — helper to save maps
- `record_bag.sh` — helper to record RGB-D + TF topics

Assumptions made
----------------
- Kinect topics: `/camera/color/image_raw`, `/camera/depth/image_raw`.
- Camera frame: `camera_link` and robot base frame: `base_link`.
- Camera is centered at `z=0.3` meters above `base_link` by default. Update
  the static transform in the launch file if the camera is at a different height.

How to tweak
------------
- Update `nav_bringup.launch.py` remappings and launch arguments to match your
  camera topics and frames.
- Fill out `nav2_params.yaml` with controller, planner, costmap, and footprint
  settings appropriate for your robot.

Host integration
----------------
If you want this module enabled for the example host `cerebellum`, a symlink
has been created at `hosts/cerebellum/modules/nav` pointing to this module.
This makes the module available when running `HOST=cerebellum ./tools/setup`.

Nav2 tuning
-----------
I added a starter `nav2_params.yaml` tuned for a small differential-drive base
similar to Create 1 (approximate 0.36m footprint). Test and tune controller
limits, costmap resolution, and planner frequency for smooth navigation.


Next steps and TODOs
--------------------
- Add `setup.sh` to install apt packages and clone any helper repos.
- Add parameter YAML files for Nav2 and RTAB-Map tuned to Create 1 sized robot.
- Integrate Kinect TF: since Kinect is centered on the robot, ensure the
  `camera_link` is published relative to `base_link` at the robot center.
- Add map saving and rosbag recording helpers for offline processing.

"""
