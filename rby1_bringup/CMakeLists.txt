cmake_minimum_required(VERSION 3.8)
project(rby1_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(robot_state_publisher REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  rclpy
  controller_manager
  hardware_interface
  robot_state_publisher
)

ament_package()
