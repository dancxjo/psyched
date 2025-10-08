cmake_minimum_required(VERSION 3.8)
project(psyched_nav)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install the python package
ament_python_install_package(${PROJECT_NAME})

# Install the single launch file and params so they are available via ament
install(
  FILES nav_bringup.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY params
  DESTINATION share/${PROJECT_NAME}/params
)

ament_package()
