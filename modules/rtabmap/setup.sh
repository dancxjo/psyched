#!/usr/bin/env bash
set -euo pipefail

case "${ROS_DISTRO}" in
	jade|kinetic|lunar|melodic|noetic)
		echo "Skipping end-of-life ROS distro: ${ROS_DISTRO}"
		exit 0
		;;
	jazzy|kilted|rolling)
		echo "Detected supported ROS2 distro: ${ROS_DISTRO}"
		;;
	*)
		echo "Proceeding with generic ROS2 distro: ${ROS_DISTRO}"
		;;
esac
echo "[rtabmap/setup] RTAB-Map is now a dedicated module/host. See README for details."
