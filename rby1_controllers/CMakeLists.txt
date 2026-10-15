cmake_minimum_required(VERSION 3.16)
project(rby1_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(DEPENDENCIES
  controller_interface
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_msgs
  tf2_ros
)
find_package(ament_cmake REQUIRED)
foreach(dep IN ITEMS ${DEPENDENCIES})
  find_package(${dep} REQUIRED)
endforeach()

add_library(rby1_controllers SHARED
  src/joint_stream_controller.cpp
  src/gpio_controllers.cpp
)
target_include_directories(rby1_controllers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(rby1_controllers PUBLIC ${DEPENDENCIES})

pluginlib_export_plugin_description_file(controller_interface rby1_controllers.xml)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS rby1_controllers
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  ament_add_gmock(test_load_controllers test/test_load_controllers.cpp)
  target_link_libraries(test_load_controllers rby1_controllers)
  ament_target_dependencies(test_load_controllers pluginlib controller_interface)
endif()

ament_export_include_directories(include)
ament_export_libraries(rby1_controllers)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES})
ament_package()
