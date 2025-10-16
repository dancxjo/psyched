cmake_minimum_required(VERSION 3.8)
project(cockpit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Call the internal macro to ensure PYTHON_INSTALL_DIR is set
_ament_cmake_python_get_python_install_dir()

# Manually copy the Python package to build directory, following symlinks
# This is needed because the default ament_python_install_package doesn't handle symlinks properly
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
     FOLLOW_SYMLINK_CHAIN
     PATTERN "__pycache__" EXCLUDE
     PATTERN "*.pyc" EXCLUDE
     PATTERN ".pytest_cache" EXCLUDE)

# Install the copied package
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}
        DESTINATION ${PYTHON_INSTALL_DIR}
        PATTERN "__pycache__" EXCLUDE
        PATTERN "*.pyc" EXCLUDE
        PATTERN ".pytest_cache" EXCLUDE)

install(PROGRAMS
  scripts/cockpit_cockpit
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

install(CODE [[
  set(_hook_dir "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/hook")

  set(_sh_file "${_hook_dir}/ros_package_path.sh")
  if(EXISTS "${_sh_file}")
    file(READ "${_sh_file}" _ros_package_path_sh)
    if(_ros_package_path_sh MATCHES "AMENT_PREFIX_PATH")
      # already patched
    else()
      file(APPEND "${_sh_file}" "\n_colcon_prepend_unique_value AMENT_PREFIX_PATH \"$COLCON_CURRENT_PREFIX\"\n")
    endif()
  endif()

  set(_ps1_file "${_hook_dir}/ros_package_path.ps1")
  if(EXISTS "${_ps1_file}")
    file(READ "${_ps1_file}" _ros_package_path_ps1)
    if(_ros_package_path_ps1 MATCHES "AMENT_PREFIX_PATH")
    else()
      file(APPEND "${_ps1_file}" "\ncolcon_prepend_unique_value AMENT_PREFIX_PATH \"$env:COLCON_CURRENT_PREFIX\"\n")
    endif()
  endif()

  set(_dsv_file "${_hook_dir}/ros_package_path.dsv")
  if(EXISTS "${_dsv_file}")
    file(READ "${_dsv_file}" _ros_package_path_dsv)
    if(_ros_package_path_dsv MATCHES "AMENT_PREFIX_PATH")
    else()
      file(APPEND "${_dsv_file}" "\nprepend-non-duplicate;AMENT_PREFIX_PATH;\n")
    endif()
  endif()
]])
