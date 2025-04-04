cmake_minimum_required(VERSION 3.5)
project(rby1_hardware)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(DEFINED ENV{RBY1_SDK_PATH})
  set(RBY1_SDK_PATH $ENV{RBY1_SDK_PATH})
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(moveit_msgs REQUIRED)

include_directories(/
  ${RBY1_SDK_PATH}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  /usr/include/eigen3 
)

link_directories(
  ${RBY1_SDK_PATH}build/src)

add_library(${PROJECT_NAME} SHARED
  src/rby1_hardware_interface.cpp
)

target_link_libraries(${PROJECT_NAME}
  rby1-sdk
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hardware_interface
  pluginlib
  moveit_msgs
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES rby1_hardware_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(hardware_interface rby1_hardware_plugins.xml)

ament_package()
