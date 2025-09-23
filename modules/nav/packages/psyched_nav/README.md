Psyched Navigation Module
=========================

This module brings up the Nav2 stack configured for AMCL-based localization on
Psyched robots equipped with a depth camera.  It intentionally avoids heavy
mapping dependencies such as RTAB-Map so the navigation pipeline stays lean in
restricted environments.

Installation
------------
Run the module setup script or install the required debs manually:

```bash
sudo apt update
sudo apt install -y \
  ros-${ROS_DISTRO}-nav2-bringup \
  ros-${ROS_DISTRO}-nav2-amcl \
  ros-${ROS_DISTRO}-nav2-controller \
  ros-${ROS_DISTRO}-nav2-costmap-2d \
  ros-${ROS_DISTRO}-nav2-navfn-planner \
  ros-${ROS_DISTRO}-nav2-smac-planner \
  ros-${ROS_DISTRO}-nav2-map-server \
  ros-${ROS_DISTRO}-nav2-lifecycle-manager \
  ros-${ROS_DISTRO}-pcl-ros \
  libpcl-dev
```

The setup script auto-detects which packages are available for the current ROS
release and installs the subset that exists in the apt repository.

Package layout
--------------
- `setup.sh` — installs the Nav2 + AMCL dependencies and links the module into
  the workspace.
- `packages/psyched_nav/nav_bringup.launch.py` — minimalist launch description
  that includes Nav2 bringup and an AMCL parameter file.
- `packages/psyched_nav/params/nav2_params.yaml` — starter Nav2 configuration
  tuned for a small differential drive base.
- `packages/psyched_nav/depth_projection.py` — utilities to turn depth images
  into `sensor_msgs/LaserScan` messages.
- `packages/psyched_nav/vision_prompt.py` — scaffolding for LLM-based scene
  annotations.

Usage notes
-----------
- Kinect topics default to `/camera/color/image_raw` and
  `/camera/depth/image_raw`; override them via launch arguments as needed.
- A static transform from `base_link` to the camera frame is published at
  `(0, 0, 0.3)`; edit the launch file if your sensor is mounted differently.
- RTAB-Map support lives in its own module.  Launch an external SLAM node if
  you need online 3D mapping in addition to localization.

Testing
-------
Run the lightweight unit tests with:

```bash
pytest modules/nav/packages/psyched_nav/tests
```
