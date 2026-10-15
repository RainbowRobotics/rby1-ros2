cmake_minimum_required(VERSION 3.16)
project(rby1_hardware_interface)

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
find_package(urdf REQUIRED)
find_package(Eigen3 REQUIRED)

# rby1-sdk C++ library, installed with scripts/install_rby1_sdk.sh (CMake package
# lib/cmake/rby1-sdk). Point CMake at it with CMAKE_PREFIX_PATH or -DRBY1_SDK_ROOT=<prefix>.
set(RBY1_SDK_ROOT "" CACHE PATH "rby1-sdk install prefix (contains lib/cmake/rby1-sdk)")
if(RBY1_SDK_ROOT)
  list(PREPEND CMAKE_PREFIX_PATH "${RBY1_SDK_ROOT}")
endif()
list(APPEND CMAKE_PREFIX_PATH "/opt/rby1-sdk" "$ENV{HOME}/.local/rby1-sdk")
find_package(rby1-sdk CONFIG REQUIRED)
message(STATUS "rby1-sdk: ${rby1-sdk_DIR}")
get_target_property(_rby1_sdk_location rby1-sdk::rby1-sdk IMPORTED_LOCATION_RELEASE)
if(_rby1_sdk_location)
  get_filename_component(RBY1_SDK_LIBRARY_DIR "${_rby1_sdk_location}" DIRECTORY)
endif()

# SDK-independent command layer (modes, fake backend): unit-testable without a robot.
add_library(rby1_command_core SHARED
  src/command_modes.cpp
  src/fake_robot_backend.cpp
)
target_include_directories(rby1_command_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
find_package(Threads REQUIRED)
target_link_libraries(rby1_command_core PUBLIC Threads::Threads)

# rby1-sdk backend.
add_library(rby1_sdk_backend SHARED
  src/sdk_robot_backend.cpp
)
target_link_libraries(rby1_sdk_backend PUBLIC rby1_command_core PRIVATE rby1-sdk::rby1-sdk Eigen3::Eigen)

# Non-real-time SDK client used by rby1_robot_manager.
add_library(rby1_robot_client SHARED
  src/rby1_robot_client.cpp
)
target_include_directories(rby1_robot_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(rby1_robot_client PRIVATE rby1-sdk::rby1-sdk Eigen3::Eigen)

# ros2_control plugin.
add_library(rby1_system_interface SHARED
  src/rby1_system_interface.cpp
)
target_link_libraries(rby1_system_interface PUBLIC rby1_command_core rby1_sdk_backend)
ament_target_dependencies(rby1_system_interface PUBLIC
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  urdf
)

# Model probe used by rby1_bringup (model:=auto).
add_executable(rby1_probe src/rby1_probe.cpp)
target_link_libraries(rby1_probe PRIVATE rby1-sdk::rby1-sdk Eigen3::Eigen)

set_target_properties(rby1_sdk_backend rby1_robot_client rby1_system_interface rby1_probe PROPERTIES
  INSTALL_RPATH "${RBY1_SDK_LIBRARY_DIR}"
  BUILD_RPATH "${RBY1_SDK_LIBRARY_DIR}"
)

pluginlib_export_plugin_description_file(hardware_interface rby1_hardware_interface.xml)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS rby1_probe RUNTIME DESTINATION lib/${PROJECT_NAME})
install(
  TARGETS rby1_command_core rby1_sdk_backend rby1_robot_client rby1_system_interface
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(rby1_command_core rby1_sdk_backend rby1_robot_client rby1_system_interface)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle urdf)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ros2_control_test_assets QUIET)

  ament_add_gtest(test_command_modes test/test_command_modes.cpp)
  target_link_libraries(test_command_modes rby1_command_core)

  ament_add_gtest(test_system_interface test/test_system_interface.cpp TIMEOUT 120 SKIP_LINKING_MAIN_LIBRARIES)
  target_link_libraries(test_system_interface rby1_system_interface)
  ament_target_dependencies(test_system_interface hardware_interface rclcpp)
endif()

ament_package()
