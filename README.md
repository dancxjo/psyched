# psyched

This is my robot control system, the software I am using to make my own robot named Pete. You can use this to make your own robot too! Start by forking this repository. Then configure the hosts in the `hosts` directory, choosing which modules to run where. ....

## Pilot module
The pilot module provides a web-based cockpit interface to control the robot. It consists of a Rust backend and a Deno frontend.

## IMU module
This is hardware specific to my robot, but it provides an example of how to create a ROS2 package that reads from an MPU6050 IMU sensor and publishes the data as ROS2 messages.

## Foot module
This is hardware specific for my iRobot Create 1 robot platform, but it provides an example of how to create a ROS2 package that interfaces with a robot platform and provides control and sensor data.

## More to come...
* ear module - microphone array and speech recognition
* eye module - camera and computer vision
* chat module - large language model integration
* voice module - text-to-speech

