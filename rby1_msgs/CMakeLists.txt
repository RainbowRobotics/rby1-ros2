cmake_minimum_required(VERSION 3.8)
project(rby1_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BrakeState.msg"
  "msg/RobotState.msg"
  "msg/ToolFlangeState.msg"
  "srv/ControlManagerCommand.srv"
  "srv/GravityCompensation.srv"
  "srv/GetLogs.srv"
  "srv/Jog.srv"
  "srv/JointOperation.srv"
  "srv/PositionPidGain.srv"
  "srv/ResetOdometry.srv"
  "srv/RobotParameter.srv"
  "srv/SetDigitalOutput.srv"
  "srv/SetLed.srv"
  "srv/SetValue.srv"
  "srv/StateOnOff.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
