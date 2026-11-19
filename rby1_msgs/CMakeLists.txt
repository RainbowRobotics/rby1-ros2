cmake_minimum_required(VERSION 3.10)
project(rby1_msgs)

# Suppress CMP0148 warning about deprecated FindPythonInterp (used by rosidl in Humble)
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/StateOnOff.srv"
  "srv/GetCartesianPose.srv"
  "srv/GravityCompensation.srv"
  "srv/ControlManagerCommand.srv"
  "srv/SetTrajectoryImpedance.srv"
  "action/Rby1JointCommand.action"
  "action/Rby1CartesianCommand.action"
  "action/StreamPosition.action"
  "action/StreamJoint.action"
  "action/StreamCartesian.action"
  "msg/BrakeState.msg"
  "msg/RobotState.msg"
  "msg/ToolFlange.msg"
  "msg/JointCommand.msg"
  "msg/CartesianCommand.msg"
  "msg/StreamJointCommand.msg"
  "msg/StreamCartesianCommand.msg"
  "msg/PartJointStates.msg"
  DEPENDENCIES std_msgs sensor_msgs action_msgs trajectory_msgs geometry_msgs
)

ament_package()
 
