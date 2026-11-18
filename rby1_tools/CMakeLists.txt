cmake_minimum_required(VERSION 3.10)
project(rby1_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(yaml-cpp REQUIRED)
# librealsense2: ROS's build (ros-humble-librealsense2) only. Intel's own package carries
# a Fast DDS of its own and exports it, which clashes with ROS's in one process, so it is
# not used. Looked for afresh every time: a build configured before ros-humble-librealsense2
# was installed remembers Intel's (realsense2_DIR in the cache) and would keep linking it.
unset(realsense2_DIR CACHE)
find_package(realsense2 QUIET PATHS /opt/ros/$ENV{ROS_DISTRO} NO_DEFAULT_PATH)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME}_core STATIC src/camera_model.cpp src/lidar_model.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(${PROJECT_NAME}_core SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(${PROJECT_NAME}_core sensor_msgs)
target_link_libraries(${PROJECT_NAME}_core yaml-cpp)

# The RealSense node needs librealsense2 (ros-humble-librealsense2); without it the rest builds.
if(realsense2_FOUND)
  add_executable(realsense_publisher src/realsense_publisher.cpp)
  target_include_directories(realsense_publisher PRIVATE ${EIGEN3_INCLUDE_DIR})
  target_link_libraries(realsense_publisher ${PROJECT_NAME}_core realsense2::realsense2)
  ament_target_dependencies(realsense_publisher rclcpp sensor_msgs geometry_msgs tf2 tf2_ros image_transport
    ament_index_cpp)
  install(TARGETS realsense_publisher DESTINATION lib/${PROJECT_NAME})
  message(STATUS "realsense_publisher: librealsense2 ${realsense2_VERSION} from ${realsense2_DIR}")
else()
  find_package(realsense2 QUIET)
  if(realsense2_FOUND)
    message(WARNING "only Intel's librealsense2 package is installed (${realsense2_DIR}), whose Fast DDS clashes with "
                    "ROS's: realsense_publisher is not built (sudo apt install ros-$ENV{ROS_DISTRO}-librealsense2)")
    unset(realsense2_DIR CACHE)
  else()
    message(WARNING "librealsense2 not found: realsense_publisher is not built "
                    "(sudo apt install ros-$ENV{ROS_DISTRO}-librealsense2)")
  endif()
endif()

# The grippers, which the driver does not drive: virtual (the state follows the commands) or
# the real one through the SDK's Dynamixel bus -- that only where the SDK is (RBY1_SDK_PATH, as
# for rby1_driver); without it gripper_driver runs virtual only.
add_executable(gripper_driver src/gripper_driver.cpp)
target_include_directories(gripper_driver PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(gripper_driver rclcpp sensor_msgs std_msgs)
# The real gripper's motors run on the tool flange's 12 V: gripper_driver switches it on through the RB-Y1
# driver's tool_flange_power service (rby1_msgs). A workspace without rby1_msgs (rby1_tools copied alone)
# builds without it, and the flange voltage is then switched on by hand.
find_package(rby1_msgs QUIET)
if(rby1_msgs_FOUND)
  target_compile_definitions(gripper_driver PRIVATE RBY1_TOOLS_WITH_MSGS)
  ament_target_dependencies(gripper_driver rby1_msgs)
else()
  message(WARNING "rby1_msgs not found: gripper_driver does not switch the tool flange's 12 V on by itself")
endif()
if(DEFINED ENV{RBY1_SDK_PATH} AND EXISTS "$ENV{RBY1_SDK_PATH}/include/rby1-sdk/base/dynamixel_bus.h")
  set(RBY1_SDK_PATH $ENV{RBY1_SDK_PATH})
  target_compile_definitions(gripper_driver PRIVATE RBY1_TOOLS_WITH_SDK)
  target_include_directories(gripper_driver PRIVATE ${RBY1_SDK_PATH}/include ${RBY1_SDK_PATH}/build/private
    ${EIGEN3_INCLUDE_DIR})
  target_link_directories(gripper_driver PRIVATE ${RBY1_SDK_PATH}/build/src)
  target_link_libraries(gripper_driver rby1-sdk)
  set_target_properties(gripper_driver PROPERTIES INSTALL_RPATH "${RBY1_SDK_PATH}/build/src")
  message(STATUS "gripper_driver: virtual and real (the RB-Y1 SDK at ${RBY1_SDK_PATH})")
else()
  message(WARNING "RBY1_SDK_PATH not set (or no SDK there): gripper_driver runs virtual only")
endif()

# The RB-Y1's LakiBeam1 2D LiDARs: their UDP packets in, a LaserScan per turn out.
add_executable(lidar_publisher src/lidar_publisher.cpp)
target_link_libraries(lidar_publisher ${PROJECT_NAME}_core)
ament_target_dependencies(lidar_publisher rclcpp sensor_msgs geometry_msgs tf2 tf2_ros)
# Their scans as one on the base (merge: true), and a one-off check of where they sit.
add_executable(lidar_merger src/lidar_merger.cpp)
target_link_libraries(lidar_merger ${PROJECT_NAME}_core)
ament_target_dependencies(lidar_merger rclcpp sensor_msgs geometry_msgs tf2 tf2_ros)
add_executable(lidar_calibrate src/lidar_calibrate.cpp)
target_link_libraries(lidar_calibrate ${PROJECT_NAME}_core)
ament_target_dependencies(lidar_calibrate rclcpp sensor_msgs)

install(TARGETS gripper_driver lidar_publisher lidar_merger lidar_calibrate
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch model DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_camera_model test/test_camera_model.cpp)
  target_link_libraries(test_camera_model ${PROJECT_NAME}_core)
  ament_add_gtest(test_lidar_model test/test_lidar_model.cpp)
  target_link_libraries(test_lidar_model ${PROJECT_NAME}_core)
  ament_add_gtest(test_gripper test/test_gripper.cpp)
  target_include_directories(test_gripper PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
endif()

ament_package()
