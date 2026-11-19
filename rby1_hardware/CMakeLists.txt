cmake_minimum_required(VERSION 3.10)
project(rby1_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# Find PythonInterp policy warning suppression
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rby1_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# The RB-Y1 SDK (https://github.com/RainbowRobotics/rby1-sdk), built from source:
# RBY1_SDK_PATH points at its source tree, with its build in <RBY1_SDK_PATH>/build.
if(NOT DEFINED ENV{RBY1_SDK_PATH})
  message(FATAL_ERROR "RBY1_SDK_PATH is not set: export RBY1_SDK_PATH=<path to rby1-sdk> (see README 1-2)")
endif()
set(RBY1_SDK_PATH $ENV{RBY1_SDK_PATH})
if(NOT EXISTS "${RBY1_SDK_PATH}/include/rby1-sdk/robot.h" OR NOT EXISTS "${RBY1_SDK_PATH}/build/src")
  message(FATAL_ERROR "RBY1_SDK_PATH=${RBY1_SDK_PATH} has no include/rby1-sdk/robot.h or build/src: build the SDK there first (see README 1-2)")
endif()

# Include directories
include_directories(
  include
  ${RBY1_SDK_PATH}/include
  ${RBY1_SDK_PATH}/build/private
  /usr/include/eigen3
)

# Export plugins
pluginlib_export_plugin_description_file(hardware_interface rby1_hardware.xml)

set(CMAKE_INSTALL_RPATH "${RBY1_SDK_PATH}/build/src")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Create hardware interface shared library
add_library(rby1_hardware SHARED
  src/rby1_system_hardware.cpp
)

# Set dependencies
ament_target_dependencies(rby1_hardware
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  rby1_msgs
  sensor_msgs
  geometry_msgs
)

# Link SDK library
target_link_directories(rby1_hardware PRIVATE ${RBY1_SDK_PATH}/build/src)
target_link_libraries(rby1_hardware rby1-sdk)

target_include_directories(rby1_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS rby1_hardware
  EXPORT export_rby1_hardware
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_rby1_hardware)
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  rby1_msgs
  sensor_msgs
  geometry_msgs
)

ament_package()
