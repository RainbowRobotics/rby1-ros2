cmake_minimum_required(VERSION 3.16)
project(rby1_gripper_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(Eigen3 REQUIRED)

set(RBY1_SDK_ROOT "" CACHE PATH "rby1-sdk install prefix (contains lib/cmake/rby1-sdk)")
if(RBY1_SDK_ROOT)
  list(PREPEND CMAKE_PREFIX_PATH "${RBY1_SDK_ROOT}")
endif()
list(APPEND CMAKE_PREFIX_PATH "/opt/rby1-sdk" "$ENV{HOME}/.local/rby1-sdk")
find_package(rby1-sdk CONFIG REQUIRED)
get_target_property(_rby1_sdk_location rby1-sdk::rby1-sdk IMPORTED_LOCATION_RELEASE)
if(_rby1_sdk_location)
  get_filename_component(RBY1_SDK_LIBRARY_DIR "${_rby1_sdk_location}" DIRECTORY)
endif()

add_library(rby1_gripper_hardware SHARED
  src/rby1_gripper_system.cpp
  src/gripper_buses.cpp
)
target_include_directories(rby1_gripper_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(rby1_gripper_hardware PRIVATE rby1-sdk::rby1-sdk Eigen3::Eigen)
ament_target_dependencies(rby1_gripper_hardware PUBLIC hardware_interface pluginlib rclcpp rclcpp_lifecycle)
set_target_properties(rby1_gripper_hardware PROPERTIES
  INSTALL_RPATH "${RBY1_SDK_LIBRARY_DIR}"
  BUILD_RPATH "${RBY1_SDK_LIBRARY_DIR}"
)

pluginlib_export_plugin_description_file(hardware_interface rby1_gripper_hardware.xml)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS rby1_gripper_hardware
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_gripper_system test/test_gripper_system.cpp)
  target_link_libraries(test_gripper_system rby1_gripper_hardware)
  ament_target_dependencies(test_gripper_system hardware_interface rclcpp)
endif()

ament_export_include_directories(include)
ament_export_libraries(rby1_gripper_hardware)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_package()
