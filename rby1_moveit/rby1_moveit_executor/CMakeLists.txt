cmake_minimum_required(VERSION 3.10)
project(rby1_moveit_executor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rby1_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(Eigen3 REQUIRED)

set(DEPENDENCIES rclcpp rclcpp_action moveit_msgs control_msgs trajectory_msgs sensor_msgs std_msgs
  geometry_msgs shape_msgs rcl_interfaces builtin_interfaces rby1_msgs urdf)

# Timing, limits and robot checks, kept apart from the node so tests need no ROS graph.
add_library(${PROJECT_NAME}_core STATIC src/trajectory.cpp src/robot.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(${PROJECT_NAME}_core ${DEPENDENCIES})

add_executable(moveit_executor src/main.cpp src/executor.cpp)
target_link_libraries(moveit_executor ${PROJECT_NAME}_core)
ament_target_dependencies(moveit_executor ${DEPENDENCIES})

add_executable(joint_state_relay src/joint_state_relay.cpp)
ament_target_dependencies(joint_state_relay rclcpp sensor_msgs)

install(TARGETS moveit_executor joint_state_relay DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_trajectory test/test_trajectory.cpp)
  target_link_libraries(test_trajectory ${PROJECT_NAME}_core)
  ament_add_pytest_test(test_launch test/test_launch.py)
endif()

ament_package()
