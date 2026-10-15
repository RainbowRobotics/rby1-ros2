cmake_minimum_required(VERSION 3.8)
project(rby1_robot_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rby1_hardware_interface REQUIRED)
find_package(rby1_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(rby1_robot_manager_node
  src/rby1_robot_manager_node.cpp
)
ament_target_dependencies(rby1_robot_manager_node
  rby1_hardware_interface
  rby1_msgs
  rclcpp
  sensor_msgs
  std_srvs
)

install(
  TARGETS rby1_robot_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
