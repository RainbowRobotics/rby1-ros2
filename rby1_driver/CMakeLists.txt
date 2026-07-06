# 할거
# 각 sdk랑 패키지 링크 중복없는지 확인 및 수정

cmake_minimum_required(VERSION 3.10)
project(rby1_driver)

# Suppress CMP0148 warning about deprecated FindPythonInterp (used by rosidl in Humble)
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(DEFINED ENV{RBY1_SDK_PATH})
  set(RBY1_SDK_PATH $ENV{RBY1_SDK_PATH})
endif()
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rby1_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(/
  ${RBY1_SDK_PATH}/include
  ${RBY1_SDK_PATH}/build/private
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  /usr/include/eigen3 
)

file(GLOB_RECURSE SOURCES src/*.cpp)

link_directories(
  ${RBY1_SDK_PATH}/build/src)

set(CMAKE_INSTALL_RPATH "${RBY1_SDK_PATH}/build/src")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

add_executable(rby1_ros2_driver
 ${SOURCES}
 src/main.cpp
)

 ament_target_dependencies(rby1_ros2_driver
 rclcpp
 rclcpp_action
 control_msgs
 sensor_msgs
 rby1_msgs
 std_srvs
 trajectory_msgs
 nav_msgs
 geometry_msgs
 tf2
 tf2_ros
)

target_link_directories(rby1_ros2_driver PRIVATE ${RBY1_SDK_PATH}/build/src)
target_link_libraries(rby1_ros2_driver
  rby1-sdk
)
target_compile_options(rby1_ros2_driver PRIVATE -Wno-maybe-uninitialized)
install(TARGETS
  rby1_ros2_driver
  DESTINATION lib/${PROJECT_NAME}
)

target_include_directories(rby1_ros2_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


# 런치 파일이 있다면 설치
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.launch.py"
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})


ament_package() 


