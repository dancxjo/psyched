cmake_minimum_required(VERSION 3.8)
project(pilot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(pilot)

install(PROGRAMS
  scripts/pilot_cockpit
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
