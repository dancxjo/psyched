cmake_minimum_required(VERSION 3.16)
project(viscera)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    scripts/viscera_monitor
    scripts/viscera_host_health
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
