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

if(BUILD_TESTING)
  # Linters (ament_lint_common); C++, Python and CMake use a 120 column limit.
  # cppcheck is skipped: Humble ships a slow version and it misreads the gtest macros.
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_cppcheck ament_cmake_cpplint ament_cmake_flake8 ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()
  ament_cpplint(MAX_LINE_LENGTH 120)
  ament_uncrustify(MAX_LINE_LENGTH 120)
endif()

ament_package()
