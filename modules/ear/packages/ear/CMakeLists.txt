cmake_minimum_required(VERSION 3.16)
project(ear)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_tests(${PROJECT_NAME}_pytest tests)
endif()

ament_package()
